# MLX90393 Driver

Embedded C driver for the **Melexis MLX90393** three-axis magnetic field sensor.

The project consists of a low-level driver for direct sensor access and an optional FreeRTOS state-machine layer for continuous interrupt-driven measurements.

The driver uses SPI communication and provides access to sensor configuration, measurement control, raw measurement data and conversion of sensor output to physical units.

## Features

- SPI communication with the MLX90393
- Sensor reset and communication probe
- Register read and write access
- Single, burst and wake-on-change measurement commands
- Reading X, Y, Z and temperature measurement data
- Magnetic field conversion to nT and uT
- Temperature conversion to milli-degrees Celsius
- Cached conversion parameters derived from the active sensor configuration
- Optional FreeRTOS state machine for continuous burst measurements
- Interrupt-driven measurement processing
- Optional formatted measurement reporting

## Files

### `mlx90393.c`

Low-level MLX90393 driver implementation.

It provides sensor commands, register access, measurement handling and conversion of raw sensor values to physical units.

### `mlx90393.h`

Public interface of the low-level driver.

It contains the driver descriptor, measurement data structure, register definitions, configuration field helpers and public API declarations.

### `mlx90393_stm.c`

FreeRTOS state-machine layer built on top of the low-level driver.

It initializes and configures the sensor, runs continuous burst measurements, handles the sensor interrupt and processes acquired measurement data.

Application-specific SPI, GPIO and measurement settings are configured in this layer.

### `mlx90393_stm.h`

Public interface of the state-machine layer.

It provides initialization and, when enabled, registration of a measurement report callback.

## Dependencies

The low-level driver depends on:

- a project-specific SPI HAL (`spi_hal`)
- FreeRTOS timing primitives used by sensor commands
- project-specific common/error handling support

The state-machine layer additionally uses FreeRTOS tasks and synchronization primitives together with project-specific GPIO, pinmux, timing, command-line and diagnostic facilities.

These dependencies are intentionally kept outside this repository and must be provided by the target project.

## Usage

An application may use the low-level driver directly when it needs explicit control over sensor configuration and measurements.

For applications requiring periodic acquisition, the state-machine layer provides a higher-level interrupt-driven operating mode built on top of the low-level API.

The exact sensor configuration, SPI connection and GPIO assignment are target-specific and are expected to be adapted in `mlx90393_stm.c`.

## License

See the license information in the source files.

## Author

Jan Rusnak  
AZTech
