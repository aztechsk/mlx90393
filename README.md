# MLX90393 Driver

Embedded C driver for the **Melexis MLX90393** three-axis magnetic field sensor.

The project consists of a low-level driver for direct sensor access and two separate FreeRTOS state machines: periodic interrupt-driven burst measurements and MCU-timed single measurements.

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
- FreeRTOS state machine for periodic burst measurements (BDR=1..63)
- Independent FreeRTOS state machine for scheduled single measurements (SM)
- Interrupt-driven measurement processing
- Optional formatted measurement reporting in uT with three decimal places
- Optional software hard-iron offset correction in nT

## Files

### `mlx90393.c`

Low-level MLX90393 driver implementation.

It provides sensor commands, register access, measurement handling and conversion of raw sensor values to physical units.

### `mlx90393.h`

Public interface of the low-level driver.

It contains the driver descriptor, measurement data structure, register definitions, configuration field helpers and public API declarations.

### `mlx90393_stm.c`

FreeRTOS state-machine layer built on top of the low-level driver.

It configures the sensor, starts periodic burst measurements, handles the data-ready
interrupt and processes acquired samples. Sensor parameters may be selected through
`sysconf.h`; SPI and GPIO connections are defined in this layer and the board configuration.

### `mlx90393_stm.h`

Public interface of the state-machine layer.

It provides initialization and, when enabled, registration of a measurement report callback.

### `mlx90393_stm_single.c` / `mlx90393_stm_single.h`

Independent FreeRTOS state machine for periodic SM acquisitions with level-high
DRDY interrupt. It issues one SM command per scheduled period, reads T/X/Y/Z
via RM, optionally applies software hard-iron correction and reports the same
text format as the burst state machine. See the header for complete timing,
configuration and callback contracts.

## Dependencies

The low-level driver depends on:

- a project-specific SPI HAL (`spi_hal`)
- FreeRTOS timing primitives used by sensor commands
- project-specific common/error handling support

The state-machine layer additionally uses FreeRTOS tasks and synchronization primitives together with project-specific GPIO, pinmux, timing, command-line and diagnostic facilities.

These dependencies are intentionally kept outside this repository and must be provided by the target project.

## Usage

The low-level driver can be used directly for sensor commands, RAM register access,
raw measurements and conversion to physical units. It supports the SB (burst), SM
(single measurement) and SW (wake-on-change) commands. The application is responsible
for configuration, scheduling and serializing access to the sensor handle.

The build compiles both STM source files, but only the selected mode defines
its task, callback and command handler:

- `MLX90393_INTR_MODE`: the existing, tested periodic burst STM
  (`mlx90393_stm.c`), using BDR=1..63; continuous burst is not supported.
- `MLX90393_SINGLE_MODE`: periodic single-measurement STM
  (`mlx90393_stm_single.c`), using SM, DRDY and `vTaskDelayUntil()`.

The application must define `MLX90393_SINGLE_MODE` as a distinct mode
identifier in `sysconf.h`. For example, if there is no conflicting identifier,
`(MLX90393_INTR_MODE + 1)` can be used. The client selects the mode in
`sysconf.h`, includes its corresponding header and initializes that STM:

```c
/* Existing burst STM */
#define MLX90393_SENSOR_MODE MLX90393_INTR_MODE
/* Application calls init_mlx90393_stm(); */
```

```c
/* New periodic single-measurement STM */
#define MLX90393_SINGLE_MODE   (MLX90393_INTR_MODE + 1)
#define MLX90393_SENSOR_MODE    MLX90393_SINGLE_MODE
#define MLX90393_MEASURE_PERIOD_MS 10U
/* Application includes mlx90393_stm_single.h, then calls init_mlx90393_stm_single(); */
```

Both STMs configure SPI mode 3 at 10 MHz, measure T/X/Y/Z, and use the
same level-high DRDY input, independent FreeRTOS task and binary semaphore.
They reset and probe the sensor, program CFG0/1/2, update the conversion cache,
and recover from sensor errors by resetting after a delay. The terminal command
`mlxs` reports diagnostics for the selected STM. Exactly one STM is active.

## STM configuration (`sysconf.h`)

The following settings are common to both STMs (the example selects burst mode):

```c
#define MLX90393_SENSOR_MODE    MLX90393_INTR_MODE
#define MLX90393_BURST_RATE_HZ  10U
#define MLX90393_RANGE_PROFILE MLX90393_RANGE_5MT
#define MLX90393_OSR            0U
#define MLX90393_DIG_FILT       4U
#define MLX90393_OSR2           0U
#define MLX90393_TCMP_EN        0U
#define MLX90393_HARDIRON_EN    0U
#define MLX90393_HARDIRON_X_NT  0
#define MLX90393_HARDIRON_Y_NT  0
#define MLX90393_HARDIRON_Z_NT  0
```

- In burst mode, `MLX90393_BURST_RATE_HZ` selects a whole-Hz **nominal upper bound**, not an
  exact output rate. The STM computes a 20 ms-step `BDR` from conversion time.
  At the defaults, a 10 Hz request produces BDR=5 and approximately 9.581 Hz.
  For this configuration, requests 1..42 Hz are accepted; the full mapping
  is documented in `mlx90393_stm.h`. Other OSR/filter settings change the rates.
- In single mode, `MLX90393_MEASURE_PERIOD_MS` is mandatory and selects the
  intended spacing of SM triggers in milliseconds (10 ms = 100 Hz). It must
  be exactly representable by FreeRTOS ticks and leave enough time for
  nominal conversion and the 140 us post-DRDY delay. If processing overruns
  the period, the STM skips missed slots rather than issuing back-to-back SM
  commands; `missed_period_cnt` tracks those slots. It does not use BDR to
  set its sample period.
- `MLX90393_RANGE_PROFILE` selects the XY digital range preset:
  `MLX90393_RANGE_5MT`, `MLX90393_RANGE_25MT`, or `MLX90393_RANGE_50MT`.
  These presets do not change the sensor's physical saturation limit.
- `MLX90393_OSR` (0..3), `MLX90393_DIG_FILT` (0..7), and `MLX90393_OSR2`
  (0..3) configure magnetic oversampling, digital filtering and temperature
  oversampling. The invalid OSR/DIG_FILT combinations (0,0), (0,1), (1,0)
  are rejected at build time for HALLCONF=0xC, in both modes.
- `MLX90393_TCMP_EN` must be `0` in either STM. On-chip temperature compensation
  is not supported here, and setting it to `1` causes a build error.
- `MLX90393_HARDIRON_EN` **must** be defined as `0` or `1` in both STMs. If enabled, the
  signed `MLX90393_HARDIRON_X_NT`, `_Y_NT` and `_Z_NT` offset constants are
  required. The STM subtracts them from converted magnetic values before
  applying `MLX90393_REPORT_FLIP_X`. No OFFSET_X/Y/Z sensor registers are used.

Reporting is controlled by `MLX90393_REPORT_SI`, `MLX90393_REPORT_FN`,
`MLX90393_REPORT_FLIP_X` and `MLX90393_REPORT_MSG_NUM`. Magnetic values are
reported in uT with three fractional digits (converted and optionally corrected
at nT resolution); temperature is reported in degrees Celsius with three
fractional digits. The report callback, when enabled, runs in the acquisition
task context. Task size and priority are supplied by `MLX90393_TASK_STACK_SIZE`
and `MLX90393_TASK_PRIO`.

The burst STM does not guarantee lossless capture of every burst conversion.
The single STM schedules one SM per accepted period but skips elapsed slots
after a delay; it does not accumulate a measurement backlog. See
`mlx90393_stm.h` and `mlx90393_stm_single.h` for their separate contracts.

## License

See the license information in the source files.

## Author

Jan Rusnak  
AZTech
