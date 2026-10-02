# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.

import utils
from drivers import *
from schema.keyboard import KeyboardUSBPort

Import("env")

keyboard = env["PIOENV"]

# Load JSON files and driver. We assume that they have been validated in `get_deps.py`.
kb_json = utils.get_kb_json(keyboard)
driver = utils.get_driver(keyboard)
driver_name = kb_json.hardware.driver

# Add source filter for driver source files
src_filter = [
    "-<hardware/>",
    f"+<hardware/{driver_name}/>",
]
# The soft-SPI driver is only needed when a pointing device is present
if kb_json.pointing_device is not None and kb_json.pointing_device.enabled:
    src_filter.append("+<hardware/spi_soft.c>")
env.Append(SRC_FILTER=src_filter)

# Build Flags
build_flags = utils.CompilerFlags()

# Include headers. We prioritize including driver and keyboard headers.
build_flags.include(f"hardware/{driver_name}")
build_flags.include(f"keyboards/{keyboard}")
build_flags.include("include")

# Bootloader Configuration
build_flags.define("BOOTLOADER_ADDR", driver.metadata.bootloader.address)
build_flags.define("BOOTLOADER_MAGIC", driver.metadata.bootloader.magic)

# Flash Configuration
flash_size = driver.metadata.flash.get_flash_size()
flash_num_sectors = driver.metadata.flash.get_num_sectors()
build_flags.define("FLASH_SIZE", flash_size)
build_flags.linker_defsym("FLASH_SIZE", flash_size)
build_flags.define("FLASH_NUM_SECTORS", flash_num_sectors)
build_flags.define("FLASH_EMPTY_VAL", driver.metadata.flash.empty_value)

match driver.metadata.flash.sector_sizes:
    case NonUniformSectors(sizes):
        build_flags.define("FLASH_SECTOR_SIZES", utils.to_c_array(sizes))
    case UniformSectors(size, _):
        build_flags.define("FLASH_SECTOR_SIZE", size)

# TinyUSB Configuration
build_flags.define("CFG_TUSB_MCU", f"OPT_MCU_{driver.tinyusb.mcu.upper()}")

# Clock Configuration
build_flags.define("BOARD_HSE_VALUE", kb_json.hardware.hse_value)
build_flags.define("HSE_VALUE", kb_json.hardware.hse_value)

# USB Configuration
if kb_json.usb.port == KeyboardUSBPort.FULL_SPEED:
    build_flags.define("BOARD_USB_FS")
else:
    build_flags.define("BOARD_USB_HS")
build_flags.define("USB_MANUFACTURER_NAME", f'"{kb_json.manufacturer}"')
build_flags.define("USB_PRODUCT_NAME", f'"{kb_json.name}"')
build_flags.define("USB_VENDOR_ID", kb_json.usb.vid)
build_flags.define("USB_PRODUCT_ID", kb_json.usb.pid)

# Analog Configuration
build_flags.define("ADC_NUM_CHANNELS", len(driver.metadata.adc.input_pins))
build_flags.define("ADC_RESOLUTION", utils.get_adc_resolution(kb_json, driver))

if kb_json.analog.invert_adc:
    build_flags.define("MATRIX_INVERT_ADC_VALUES")

if kb_json.analog.delay is not None:
    build_flags.define("ADC_SAMPLE_DELAY", kb_json.analog.delay)


def define_analog_macros(analog, suffix="", pad_width=None):
    # Raw ADC Input Configuration
    if analog.raw is not None:
        raw = analog.raw
        build_flags.define(f"ADC_NUM_RAW_INPUTS{suffix}", len(raw.input))
        build_flags.define(
            f"ADC_RAW_INPUT_CHANNELS{suffix}",
            utils.to_c_array(driver.metadata.adc.to_adc_inputs(raw.input)),
        )
        build_flags.define(
            f"ADC_RAW_INPUT_VECTOR{suffix}", utils.to_c_array(raw.vector)
        )

    # Analog Multiplexer ADC Input Configuration
    if analog.mux is not None:
        mux = analog.mux
        build_flags.define(f"ADC_NUM_MUX_INPUTS{suffix}", len(mux.input))
        build_flags.define(
            f"ADC_MUX_INPUT_CHANNELS{suffix}",
            utils.to_c_array(driver.metadata.adc.to_adc_inputs(mux.input)),
        )
        build_flags.define(f"ADC_NUM_MUX_SELECT_PINS{suffix}", len(mux.select))

        ports, pin_nums = driver.metadata.adc.to_gpio_array(mux.select)
        build_flags.define(f"ADC_MUX_SELECT_PORTS{suffix}", utils.to_c_array(ports))
        build_flags.define(f"ADC_MUX_SELECT_PINS{suffix}", utils.to_c_array(pin_nums))

        transposed = list(map(list, zip(*mux.matrix)))
        if pad_width is not None:
            for row in transposed:
                row.extend([0] * (pad_width - len(row)))
        build_flags.define(
            f"ADC_MUX_INPUT_MATRIX{suffix}",
            utils.to_c_array(transposed),
        )


if kb_json.split is not None and kb_json.split.enabled:
    split = kb_json.split
    analog_left = split.analog_left if split.analog_left is not None else kb_json.analog
    analog_right = (
        split.analog_right if split.analog_right is not None else kb_json.analog
    )

    # Both halves must use the same analog configuration structure (raw and/or mux)
    # so that the driver can use a common local key mapping.
    if (analog_left.raw is not None) != (analog_right.raw is not None):
        raise ValueError(
            "Both split halves must either have raw analog inputs or neither"
        )
    if (analog_left.mux is not None) != (analog_right.mux is not None):
        raise ValueError(
            "Both split halves must either have mux analog inputs or neither"
        )

    if analog_left.mux is not None:
        if len(analog_left.mux.select) != len(analog_right.mux.select):
            raise ValueError(
                "Left and right mux select pin counts must match"
            )

    # For split keyboards, the unsuffixed macros use the maximum left/right
    # counts so shared buffers and matrix dimensions are sized for either half.
    # The suffixed macros keep each half's actual count, but matrices are padded
    # to the maximum width so the C array type is consistent.
    max_mux_inputs = 0
    max_raw_inputs = 0
    if analog_left.mux is not None:
        max_mux_inputs = max(
            len(analog_left.mux.input), len(analog_right.mux.input)
        )
    if analog_left.raw is not None:
        max_raw_inputs = max(
            len(analog_left.raw.input), len(analog_right.raw.input)
        )

    # Define base macros from the left half, with maximum counts and padded
    # matrices, so non-half-specific code has a stable sizing type.
    if analog_left.mux is not None:
        mux = analog_left.mux
        build_flags.define("ADC_NUM_MUX_INPUTS", max_mux_inputs)
        # Pad the channel array to the maximum count so its length matches
        # ADC_NUM_MUX_INPUTS. Padded entries are never indexed.
        mux_channels = driver.metadata.adc.to_adc_inputs(mux.input)
        mux_channels.extend([0] * (max_mux_inputs - len(mux_channels)))
        build_flags.define(
            "ADC_MUX_INPUT_CHANNELS",
            utils.to_c_array(mux_channels),
        )
        build_flags.define("ADC_NUM_MUX_SELECT_PINS", len(mux.select))
        ports, pin_nums = driver.metadata.adc.to_gpio_array(mux.select)
        build_flags.define("ADC_MUX_SELECT_PORTS", utils.to_c_array(ports))
        build_flags.define("ADC_MUX_SELECT_PINS", utils.to_c_array(pin_nums))
        transposed = list(map(list, zip(*mux.matrix)))
        for row in transposed:
            row.extend([0] * (max_mux_inputs - len(row)))
        build_flags.define(
            "ADC_MUX_INPUT_MATRIX",
            utils.to_c_array(transposed),
        )

    if analog_left.raw is not None:
        raw = analog_left.raw
        build_flags.define("ADC_NUM_RAW_INPUTS", max_raw_inputs)
        # Pad the channel and vector arrays to the maximum count so their
        # lengths match ADC_NUM_RAW_INPUTS. Padded entries are never indexed.
        raw_channels = driver.metadata.adc.to_adc_inputs(raw.input)
        raw_channels.extend([0] * (max_raw_inputs - len(raw_channels)))
        build_flags.define(
            "ADC_RAW_INPUT_CHANNELS",
            utils.to_c_array(raw_channels),
        )
        raw_vector = list(raw.vector)
        raw_vector.extend([0] * (max_raw_inputs - len(raw_vector)))
        build_flags.define(
            "ADC_RAW_INPUT_VECTOR", utils.to_c_array(raw_vector)
        )

    # Define per-half macros with padded matrices.
    define_analog_macros(analog_left, "_LEFT", pad_width=max_mux_inputs)
    define_analog_macros(analog_right, "_RIGHT", pad_width=max_mux_inputs)
else:
    # Generate the base analog macros used by non-split keyboards.
    define_analog_macros(kb_json.analog)

# Calibration Configuration
build_flags.define(
    "DEFAULT_CALIBRATION", utils.to_c_struct(kb_json.calibration.model_dump())
)

# Wear leveling configuration
wear_leveling = kb_json.wear_leveling
wl_virtual_size = (wear_leveling and wear_leveling.virtual_size) or 8192
wl_write_log_size = (wear_leveling and wear_leveling.write_log_size) or 65536
wl_backing_store_size = wl_virtual_size + wl_write_log_size

build_flags.define("WL_VIRTUAL_SIZE", wl_virtual_size)
build_flags.define("WL_WRITE_LOG_SIZE", wl_write_log_size)

# Reserve flash for wear leveling (round up to whole sectors from the end)
wl_base_address = flash_size - driver.metadata.flash.round_up_to_flash_sectors(
    wl_backing_store_size
)
build_flags.define("WL_BASE_ADDRESS", wl_base_address)
build_flags.linker_defsym("WL_BASE_ADDRESS", wl_base_address)

# Keyboard Configuration
kb = kb_json.keyboard
build_flags.define("NUM_PROFILES", kb.num_profiles)
build_flags.define("NUM_LAYERS", kb.num_layers)
build_flags.define("NUM_KEYS", kb.num_keys)
build_flags.define("NUM_ADVANCED_KEYS", kb.num_advanced_keys)
build_flags.define(
    "NUM_DYNAMIC_KEYSTROKE_MAX_BINDINGS", kb.num_dynamic_keystroke_max_bindings
)
build_flags.define("NUM_MACRO_NODES", kb.num_macro_nodes)

# Default Keymaps (per profile)
default_keymaps = utils.resolve_default_keymaps(kb_json)
build_flags.define("DEFAULT_KEYMAPS", utils.to_c_array(default_keymaps))

# Default advanced keys (e.g. modtap entries from keyboard.json), applied to
# every profile on reset. Unspecified slots default to AK_TYPE_NONE.
def advanced_key_to_c(ak):
    if ak.type == "tap_hold":
        return (
            "{ .layer = %d, .key = %d, .type = AK_TYPE_TAP_HOLD, "
            ".tap_hold = { .tap_keycode = %s, .hold_keycode = %s, "
            ".tapping_term = %d, .hold_on_other_key_press = %s } }"
            % (
                ak.layer,
                ak.key,
                ak.tap_keycode,
                ak.hold_keycode,
                ak.tapping_term,
                "true" if ak.hold_on_other_key_press else "false",
            )
        )
    raise ValueError(f"Unsupported advanced key type: {ak.type}")


if kb_json.advanced_keys:
    entries = ", ".join(advanced_key_to_c(ak) for ak in kb_json.advanced_keys)
    build_flags.define("DEFAULT_ADVANCED_KEYS", "{ " + entries + " }")

# Actuation Configuration
if kb_json.actuation is not None:
    actuation = kb_json.actuation
    if actuation.actuation_point is not None:
        # FW reads DEFAULT_ACTUATION_POINT; keep the macro name aligned.
        build_flags.define("DEFAULT_ACTUATION_POINT", actuation.actuation_point)

# Nominal travel domain (0.01mm units). Default 400 (=4.00mm).
travel_mm = kb_json.keyboard.travel_mm
if travel_mm is not None:
    travel_units = int(round(float(travel_mm) * 100.0))
    if travel_units <= 0 or travel_units > 65535:
        raise ValueError(f"travel_mm out of range: {travel_mm}")
    build_flags.define("TRAVEL_UNITS", travel_units)

# Split Keyboard Configuration
if kb_json.split is not None and kb_json.split.enabled:
    split = kb_json.split

    build_flags.define("SPLIT_KEYBOARD")
    build_flags.define("SPLIT_UART_INSTANCE", split.uart_instance)
    build_flags.define("SPLIT_UART_BAUD_RATE", split.baud_rate)
    build_flags.define("SPLIT_NUM_KEYS_LEFT", split.left_keys)
    build_flags.define("SPLIT_NUM_KEYS_RIGHT", split.right_keys)
    build_flags.define(
        "DEFAULT_SPLIT_HANDEDNESS",
        0 if split.eeprom_default_handedness == "left" else 1,
    )

    left_offset = split.left_key_offset
    right_offset = split.right_key_offset

    if left_offset + split.left_keys > kb_json.keyboard.num_keys:
        raise ValueError(
            "Split left half key range exceeds keyboard.num_keys"
        )
    if right_offset + split.right_keys > kb_json.keyboard.num_keys:
        raise ValueError(
            "Split right half key range exceeds keyboard.num_keys"
        )
    if left_offset < right_offset + split.right_keys and right_offset < left_offset + split.left_keys:
        raise ValueError("Split left and right key ranges overlap")

    build_flags.define("SPLIT_KEY_OFFSET_LEFT", left_offset)
    build_flags.define("SPLIT_KEY_OFFSET_RIGHT", right_offset)

    local_keys_max = max(split.left_keys, split.right_keys)
    build_flags.define("SPLIT_NUM_KEYS_LOCAL_MAX", local_keys_max)

    if split.uart_tx_pin is None:
        raise ValueError("uart_tx_pin is required when split is enabled")

    # UART TX pin
    if split.uart_tx_pin is not None:
        tx_port, tx_pin = utils.pin_name_to_port_pin(split.uart_tx_pin)
        build_flags.define("SPLIT_UART_TX_PORT", tx_port)
        build_flags.define("SPLIT_UART_TX_PIN", tx_pin)
        build_flags.define("SPLIT_UART_TX_MUX", split.uart_tx_mux)

    # UART RX pin (full-duplex)
    if split.uart_rx_pin is not None:
        rx_port, rx_pin = utils.pin_name_to_port_pin(split.uart_rx_pin)
        build_flags.define("SPLIT_UART_RX_PORT", rx_port)
        build_flags.define("SPLIT_UART_RX_PIN", rx_pin)
        build_flags.define("SPLIT_UART_RX_MUX", split.uart_rx_mux)
    else:
        # Default to half-duplex if no RX pin is specified
        build_flags.define("SPLIT_UART_HALF_DUPLEX")

    # Handedness detection
    if split.handedness == "pin":
        if split.handedness_pin is None:
            raise ValueError("handedness_pin is required when handedness is 'pin'")
        h_port, h_pin = utils.pin_name_to_port_pin(split.handedness_pin)
        build_flags.define("SPLIT_HANDEDNESS_PIN")
        build_flags.define("SPLIT_HANDEDNESS_PIN_PORT", h_port)
        build_flags.define("SPLIT_HANDEDNESS_PIN_PIN", h_pin)
        if split.handedness_pin_low_is_left:
            build_flags.define("SPLIT_HANDEDNESS_PIN_LOW_IS_LEFT")
    elif split.handedness == "eeprom":
        build_flags.define("SPLIT_HANDEDNESS_EEPROM")
    elif split.handedness == "left":
        build_flags.define("SPLIT_HANDEDNESS_LEFT")
    elif split.handedness == "right":
        build_flags.define("SPLIT_HANDEDNESS_RIGHT")
    elif split.handedness == "usb":
        build_flags.define("SPLIT_HANDEDNESS_USB")

# Pointing Device Configuration
if kb_json.pointing_device is not None and kb_json.pointing_device.enabled:
    pd = kb_json.pointing_device

    if pd.sensor != "pmw3610":
        raise ValueError(f"Unsupported pointing device sensor: {pd.sensor}")

    build_flags.define("POINTING_DEVICE_ENABLED")
    build_flags.define("POINTING_DEVICE_SENSOR_PMW3610")

    if pd.side == "both":
        # Dual-sensor build: both halves carry a sensor on the same wiring.
        build_flags.define("POINTING_DEVICE_SIDE_LEFT")
        build_flags.define("POINTING_DEVICE_SIDE_RIGHT")
    elif pd.side == "left":
        build_flags.define("POINTING_DEVICE_SIDE_LEFT")
    else:
        build_flags.define("POINTING_DEVICE_SIDE_RIGHT")

    cs_port, cs_pin = utils.pin_name_to_port_pin(pd.pins.cs)
    sck_port, sck_pin = utils.pin_name_to_port_pin(pd.pins.sck)
    mosi_port, mosi_pin = utils.pin_name_to_port_pin(pd.pins.mosi)
    miso_port, miso_pin = utils.pin_name_to_port_pin(pd.pins.miso)

    build_flags.define("PMW3610_CS_PORT", cs_port)
    build_flags.define("PMW3610_CS_PIN", cs_pin)
    build_flags.define("PMW3610_SCK_PORT", sck_port)
    build_flags.define("PMW3610_SCK_PIN", sck_pin)
    build_flags.define("PMW3610_MOSI_PORT", mosi_port)
    build_flags.define("PMW3610_MOSI_PIN", mosi_pin)
    build_flags.define("PMW3610_MISO_PORT", miso_port)
    build_flags.define("PMW3610_MISO_PIN", miso_pin)

    if pd.pins.irq is not None:
        irq_port, irq_pin = utils.pin_name_to_port_pin(pd.pins.irq)
        build_flags.define("PMW3610_IRQ_PIN")
        build_flags.define("PMW3610_IRQ_PORT", irq_port)
        build_flags.define("PMW3610_IRQ_PIN_NUM", irq_pin)

    if pd.cpi % 200 != 0:
        raise ValueError("PMW3610 CPI must be a multiple of 200")

    build_flags.define("PMW3610_CPI", pd.cpi)

    # Convert angle to swap/invert, then XOR with explicit user overrides
    angle_swap, angle_inv_x, angle_inv_y = False, False, False
    if pd.angle == 0:
        angle_swap, angle_inv_x, angle_inv_y = False, False, False
    elif pd.angle == 90:
        angle_swap, angle_inv_x, angle_inv_y = True, True, False
    elif pd.angle == 180:
        angle_swap, angle_inv_x, angle_inv_y = False, True, True
    elif pd.angle == 270:
        angle_swap, angle_inv_x, angle_inv_y = True, False, True
    else:
        raise ValueError("pointing_device.angle must be 0, 90, 180, or 270")

    final_swap = angle_swap ^ pd.swap_xy
    final_inv_x = angle_inv_x ^ pd.invert_x
    final_inv_y = angle_inv_y ^ pd.invert_y

    # Axis orientation is applied at runtime in pointing_device.c from the
    # persisted pointing configuration; seed its defaults so the out-of-box
    # behavior matches the keyboard.json values.
    build_flags.define("DEFAULT_POINTING_SWAP_AXES", "true" if final_swap else "false")
    build_flags.define("DEFAULT_POINTING_INVERT_X", "true" if final_inv_x else "false")
    build_flags.define("DEFAULT_POINTING_INVERT_Y", "true" if final_inv_y else "false")

    build_flags.define("POINTING_DEVICE_X_SCALE_PERCENT", pd.x_scale)
    build_flags.define("POINTING_DEVICE_Y_SCALE_PERCENT", pd.y_scale)

    if pd.auto_mouse_layer >= 0:
        build_flags.define("POINTING_DEVICE_AUTO_MOUSE_LAYER", pd.auto_mouse_layer)

    if pd.scroll_layer >= 0:
        build_flags.define("POINTING_DEVICE_SCROLL_LAYER", pd.scroll_layer)

# Add source build flags
env.Append(BUILD_FLAGS=build_flags.get_flags())
