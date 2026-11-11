# Example for `bmi160` driver default use

## What it does

Example configures a BMI160 device.

It shows tap detection feature.

## Wiring

Connect `SCL` and `SDA` pins to the following pins with appropriate pull-up
resistors.

| Name | Description | Defaults |
|------|-------------|----------|
| `CONFIG_EXAMPLE_I2C_MASTER_SCL` | GPIO number for `SCL` | "6" for `esp32c3`, "19" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_I2C_MASTER_SDA` | GPIO number for `SDA` | "5" for `esp32c3`, "18" for `esp32`, `esp32s2`, and `esp32s3` |
| `CONFIG_EXAMPLE_INT1_GPIO` | GPIO number for `INT1` | "5" for `esp32`, `esp32s2`, and `esp32s3`, "2" for `esp32c2`, `esp32c3`, `esp32c5`, `esp32c6`, `esp32c61`, `esp32h2`, and `esp32p4` |

## Notes
- Choose I2C address under `Example configuration` in `menuconfig`. The default is
  `BMI160_I2C_ADDRESS_VDD` (`ADDR` pin is connected to VDD).
