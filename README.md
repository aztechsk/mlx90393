# MLX90393 Driver

Driver for the Melexis MLX90393 three-axis magnetic field sensor with SPI interface.

The driver provides sensor initialization, measurement control, register access, reset and NVRAM operations, together with conversion of raw magnetic and temperature data to physical units.

## Features

- SPI communication through `spi_hal`
- Single, burst and wake-on-change measurement modes
- X/Y/Z magnetic field and temperature readout
- Configuration register read/write access
- Soft reset
- Configuration recall from and store to NVRAM
- Basic communication probe
- Cached conversion parameters derived from the active sensor configuration
- Magnetic field conversion to nT and uT
- Temperature conversion to milli-degrees Celsius
- Support for gain, resolution, `HALLCONF` and `TCMP_EN` dependent magnetic scaling

## Files

- `mlx90393.c` - driver implementation
- `mlx90393.h` - public interface, register definitions and conversion helpers

## Dependencies

The driver is intended for an embedded FreeRTOS environment and uses the project-specific SPI HAL and utility modules.

## Author

Jan Rusnak  
Copyright (c) 2025 AZTech
