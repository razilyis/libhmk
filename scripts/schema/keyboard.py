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

from enum import Enum
from typing import Literal
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    model_validator,
)


# Base model that rejects unknown fields instead of silently ignoring them
class StrictBaseModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class KeyboardUSBPort(str, Enum):
    FULL_SPEED = "fs"
    HIGH_SPEED = "hs"


# USB Configuration
class KeyboardUSB(StrictBaseModel):
    # USB Vendor ID
    vid: str = Field(pattern=r"^0x[0-9A-Fa-f]{4}$")
    # USB Product ID
    pid: str = Field(pattern=r"^0x[0-9A-Fa-f]{4}$")
    port: KeyboardUSBPort


# Keyboard Configuration
class KeyboardKeyboard(StrictBaseModel):
    num_profiles: int = Field(ge=1, le=8)
    num_layers: int = Field(ge=1, le=8)
    num_keys: int = Field(ge=1, le=256)
    num_advanced_keys: int = Field(ge=1, le=64)
    # Maximum number of Dynamic Keystroke bindings per key. Higher values may require higher storage sizes.
    num_dynamic_keystroke_max_bindings: int = Field(ge=4, le=64, default=4)
    num_macro_nodes: int = Field(ge=1, le=255, default=128)
    # Nominal full key travel in millimeters. Converted to TRAVEL_UNITS (x100).
    travel_mm: float | None = Field(default=None, gt=0, le=655.35)


# Hardware Configuration
class KeyboardHardware(StrictBaseModel):
    # High-speed external oscillator value in Hz
    hse_value: PositiveInt
    # Keyboard driver name
    driver: str


# Raw ADC input configuration
class KeyboardAnalogRaw(StrictBaseModel):
    # Array of raw ADC input channels. If a string is provided, it is used as the GPIO pin name
    input: list[NonNegativeInt | str]
    # Array of key index mappings for the raw ADC inputs
    vector: list[NonNegativeInt]


# Analog Multiplexer ADC input configuration
class KeyboardAnalogMux(StrictBaseModel):
    # Array of GPIO pin names for the multiplexer select lines
    select: list[str]
    # Array of ADC input channels for the multiplexers. If a string is provided, it is used as the GPIO pin name
    input: list[NonNegativeInt | str]
    # Mapping from multiplexers to ADC input channels
    matrix: list[list[NonNegativeInt]]


# Analog Configuration
class KeyboardAnalog(StrictBaseModel):
    # ADC resolution for this keyboard. A higher value means higher accuracy but slower matrix scans. Default to the maximum resolution supported by the MCU if not provided.
    adc_resolution: PositiveInt | None = None
    # Set to true if the ADC value is inversely proportional to the travel distance of the keys
    invert_adc: bool = False
    # Delay in microseconds between ADC scans
    delay: int | None = None
    raw: KeyboardAnalogRaw | None = None
    mux: KeyboardAnalogMux | None = None


# Calibration Configuration
class KeyboardCalibration(StrictBaseModel):
    # See `include/lib/eeconfig.h`
    initial_rest_value: NonNegativeInt
    # See `include/lib/eeconfig.h`
    initial_bottom_out_threshold: NonNegativeInt


# Wear leveling Configuration
class KeyboardWearLeveling(StrictBaseModel):
    # Size of the virtual persistent storage in bytes. There must be enough RAM of this size to hold the entire virtual storage.
    virtual_size: int = Field(ge=1, le=16384)
    # Size of the write log in bytes
    write_log_size: int = Field(ge=1, le=65536)


class KeyboardLayoutKey(StrictBaseModel):
    key: NonNegativeInt
    w: PositiveFloat | None = None
    h: PositiveFloat | None = None
    x: float | None = None
    y: float | None = None
    # Secondary geometry for option keys (KLE-style stepped/ISO keys)
    w2: PositiveFloat | None = None
    h2: PositiveFloat | None = None
    x2: float | None = None
    y2: float | None = None
    # Option key and value pairs for the corresponding labels
    option: tuple[int, int] | None = None


# Keyboard Layout
class KeyboardLayout(StrictBaseModel):
    # Labels for each layout option. Use a string for a toggle option, or an array for a select option
    labels: list[str | list[str]] | None = None
    # Metadata for how the keyboard should be rendered in the web configurator
    keymap: list[list[KeyboardLayoutKey]]


# Per-half analog configuration for split keyboards
class KeyboardSplitAnalog(StrictBaseModel):
    raw: KeyboardAnalogRaw | None = None
    mux: KeyboardAnalogMux | None = None


# Split Keyboard Configuration
class KeyboardSplit(StrictBaseModel):
    # Enable split keyboard support
    enabled: bool = False
    # UART peripheral instance (e.g., 1 for USART1, 2 for USART2)
    uart_instance: int = Field(default=1, ge=1, le=8)
    # UART TX pin (half-duplex) or TX pin (full-duplex)
    uart_tx_pin: str | None = None
    # GPIO alternate function mux for TX pin (0-15)
    uart_tx_mux: int = Field(default=7, ge=0, le=15)
    # UART RX pin for full-duplex operation
    uart_rx_pin: str | None = None
    # GPIO alternate function mux for RX pin (0-15)
    uart_rx_mux: int = Field(default=7, ge=0, le=15)
    # UART baud rate
    baud_rate: int = Field(default=1000000, ge=9600, le=10000000)
    # Handedness detection method
    handedness: Literal["pin", "eeprom", "left", "right", "usb"] = "left"
    # GPIO pin for handedness detection (high = left, low = right unless inverted)
    handedness_pin: str | None = None
    # Number of keys connected to the left half
    left_keys: int = Field(default=1, ge=1, le=256)
    # Number of keys connected to the right half
    right_keys: int = Field(default=1, ge=1, le=256)
    # Key index offset of the left half in the global keymap
    left_key_offset: int = Field(default=0, ge=0, le=255)
    # Key index offset of the right half in the global keymap
    right_key_offset: int = Field(default=0, ge=0, le=255)
    # Invert handedness pin logic (low = left, high = right)
    handedness_pin_low_is_left: bool = False
    # Default handedness when using EEPROM detection (left/right)
    eeprom_default_handedness: Literal["left", "right"] = "left"
    # Analog configuration for the left half (falls back to global analog if omitted)
    analog_left: KeyboardSplitAnalog | None = None
    # Analog configuration for the right half (falls back to global analog if omitted)
    analog_right: KeyboardSplitAnalog | None = None


# Actuation Configuration
class KeyboardActuation(StrictBaseModel):
    # Default actuation point in TRAVEL_UNITS (0.01mm nominal)
    actuation_point: int = Field(ge=0, le=65535)


# Default Tap-Hold (modtap) advanced key entry. Keycodes are C identifiers
# from `include/keycodes.h` (e.g. "KC_A", "KC_LEFT_SHIFT"), resolved at
# firmware compile time.
class KeyboardAdvancedKeyTapHold(StrictBaseModel):
    # Advanced key type
    type: Literal["tap_hold"]
    # Layer of the key to bind
    layer: NonNegativeInt
    # Zero-based global key index
    key: NonNegativeInt
    # Keycode registered when the key is tapped
    tap_keycode: str
    # Keycode registered while the key is held
    hold_keycode: str
    # Tapping term in milliseconds
    tapping_term: int = Field(default=200, ge=0, le=65535)
    # Immediately resolve to the hold action when another key is pressed
    hold_on_other_key_press: bool = False


# Pointing Device Pin Configuration
class KeyboardPointingDevicePins(StrictBaseModel):
    # Chip-select GPIO pin name
    cs: str
    # SPI clock GPIO pin name
    sck: str
    # SPI MOSI GPIO pin name
    mosi: str
    # SPI MISO GPIO pin name
    miso: str
    # Optional motion interrupt GPIO pin name
    irq: str | None = None


# Pointing Device Configuration
class KeyboardPointingDevice(StrictBaseModel):
    # Enable pointing device support
    enabled: bool = False
    # Sensor type. Currently only "pmw3610" is supported.
    sensor: str = Field(default="pmw3610", pattern=r"^(pmw3610)$")
    # Which half the sensor is connected to (only used for split keyboards).
    # "both" mounts a sensor on each half with the same wiring/pins, enabling
    # the dual-sensor firmware build (POINTING_DEVICE_SIDE_LEFT +
    # POINTING_DEVICE_SIDE_RIGHT). Per-side orientation tuning stays a runtime
    # setting through SET_SIDE_CONFIG.
    side: Literal["left", "right", "both"] = "right"
    # GPIO pin assignments
    pins: KeyboardPointingDevicePins
    # CPI (counts per inch). PMW3610 supports 200-3200 in steps of 200.
    cpi: int = Field(default=800, ge=200, le=3200)
    # Sensor rotation angle in degrees.
    angle: Literal[0, 90, 180, 270] = 0
    # Swap X and Y axes (applied after angle rotation)
    swap_xy: bool = False
    # Invert X axis
    invert_x: bool = False
    # Invert Y axis
    invert_y: bool = False
    # Per-axis gain in percent, applied after orientation on the sensing half.
    # Compensates a sensor that reads one axis short (e.g. a sensor tilted
    # against the ball surface).
    x_scale: int = Field(default=100, ge=25, le=400)
    y_scale: int = Field(default=100, ge=25, le=400)
    # Layer to temporarily activate when the sensor is moved (-1 to disable)
    auto_mouse_layer: int = Field(default=-1, ge=-1, le=7)
    # While this layer is active, pointer movement is sent as scroll (wheel)
    # input instead of cursor movement (-1 to disable)
    scroll_layer: int = Field(default=-1, ge=-1, le=7)


# keyboard.json Schema
class Keyboard(StrictBaseModel):
    name: str
    manufacturer: str
    maintainer: str
    usb: KeyboardUSB
    keyboard: KeyboardKeyboard
    hardware: KeyboardHardware
    analog: KeyboardAnalog
    calibration: KeyboardCalibration
    wear_leveling: KeyboardWearLeveling | None = None
    split: KeyboardSplit | None = None
    layout: KeyboardLayout
    # Default keymap
    keymap: list[list[str]] | None = None
    # Default keymaps for each profile. If not specified, the default keymap will be used for all profiles.
    keymaps: list[list[list[str]]] | None = None
    actuation: KeyboardActuation | None = None
    pointing_device: KeyboardPointingDevice | None = None
    # Default advanced keys (e.g. modtap entries) applied to every profile
    advanced_keys: list[KeyboardAdvancedKeyTapHold] | None = None

    @model_validator(mode="after")
    def check_analog_config(self) -> "Keyboard":
        split_enabled = self.split is not None and self.split.enabled
        if not split_enabled:
            # Non-split keyboards read the top-level analog section, so it
            # must define at least one ADC input source.
            if self.analog.raw is None and self.analog.mux is None:
                raise ValueError(
                    "analog.raw or analog.mux is required for non-split keyboards"
                )
        else:
            # Split keyboards read analog_left/analog_right (falling back to
            # the top-level analog section), so each half must end up with at
            # least one ADC input source.
            analog_left = (
                self.split.analog_left
                if self.split.analog_left is not None
                else self.analog
            )
            analog_right = (
                self.split.analog_right
                if self.split.analog_right is not None
                else self.analog
            )
            for name, analog in (("left", analog_left), ("right", analog_right)):
                if analog.raw is None and analog.mux is None:
                    raise ValueError(
                        f"analog.raw or analog.mux is required for the {name} half"
                    )
        return self

    @model_validator(mode="after")
    def check_advanced_keys(self) -> "Keyboard":
        if self.advanced_keys is None:
            return self
        if len(self.advanced_keys) > self.keyboard.num_advanced_keys:
            raise ValueError(
                f"advanced_keys has {len(self.advanced_keys)} entries, but "
                f"keyboard.num_advanced_keys is {self.keyboard.num_advanced_keys}"
            )
        for ak in self.advanced_keys:
            if ak.layer >= self.keyboard.num_layers:
                raise ValueError(
                    f"advanced_keys layer ({ak.layer}) must be less than "
                    f"keyboard.num_layers ({self.keyboard.num_layers})"
                )
            if ak.key >= self.keyboard.num_keys:
                raise ValueError(
                    f"advanced_keys key ({ak.key}) must be less than "
                    f"keyboard.num_keys ({self.keyboard.num_keys})"
                )
        return self
