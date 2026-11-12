# MLX90393 Driver

Embedded C driver for the **Melexis MLX90393** three-axis magnetic field sensor.

The project consists of a low-level driver for direct sensor access and three independent FreeRTOS state machines: periodic interrupt-driven burst,
MCU-timed single measurements and continuous burst (BDR=0).

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
- Independent FreeRTOS state machine for continuous burst measurements (BDR=0)
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

### `mlx90393_stm_cont.c` / `mlx90393_stm_cont.h`

Independent FreeRTOS state machine for continuous burst acquisition (BDR=0).
One SB command starts autonomous T/X/Y/Z conversions, with DRDY interrupt
and RM readout after each available sample. The conversion rate is set by
OSR/DIG_FILT/OSR2, not by a requested sampling period. See the header for
DRDY handling, data-loss limitations and configuration details.

## Dependencies

The low-level driver depends on:

- a project-specific SPI HAL (`spi_hal`)
- FreeRTOS timing primitives used by sensor commands
- project-specific common/error handling support

The state-machine layer additionally uses FreeRTOS tasks and synchronization primitives together with project-specific GPIO, pinmux, timing, command-line and diagnostic facilities.

These dependencies are intentionally kept outside this repository and must be provided by the target project.

## Client guide: using the acquisition STMs

The three STMs are alternative application-facing acquisition engines. Each
owns the SPI sensor handle, a dedicated FreeRTOS task and the DRDY interrupt.
The client chooses **one** STM at build time, calls its initialization function
once during startup and receives completed samples through logging and/or
optional text and binary callbacks. It does not issue sensor commands for every
measurement. The STMs have no sample queue; the binary callback exposes both
original sensor RAW and driver-processed nT/mC values.
Do not access the same sensor concurrently through the LL API while an STM
is active: the STM owns its sensor handle and command sequence. For direct
LL use, the application provides its own scheduling and serialization.

### Choosing the acquisition mode

| Client requirement | Periodic Burst | Periodic Single | Continuous Burst |
| --- | --- | --- | --- |
| Mode selector | `MLX90393_INTR_MODE` | `MLX90393_SINGLE_MODE` | `MLX90393_CONT_MODE` |
| STM source | `mlx90393_stm.c` | `mlx90393_stm_single.c` | `mlx90393_stm_cont.c` |
| Initialization | `init_mlx90393_stm()` | `init_mlx90393_stm_single()` | `init_mlx90393_stm_cont()` |
| Who starts conversions? | Sensor after one `SB` | MCU: one `SM` each period | Sensor after one `SB` |
| Who sets the period? | Sensor `BDR`, calculated from requested Hz | FreeRTOS `vTaskDelayUntil()` | Sensor conversion time (`BDR=0`) |
| Client rate parameter | `MLX90393_BURST_RATE_HZ` | `MLX90393_MEASURE_PERIOD_MS` | None |
| Example nominal rate | ~9.581 Hz for a 10 Hz request | 100 Hz requested with a 10 ms period | ~249 Hz sensor rate |
| After a measurement | Sensor waits for its next burst interval | Sensor is idle until the next `SM` | Sensor starts its next conversion |
| When processing is late | Samples may be overwritten | Missed trigger slots are skipped | Samples may be overwritten |
| Typical use | Moderate-rate autonomous acquisition | MCU-controlled periodic sampling | Maximum autonomous TXYZ throughput |

All three STMs measure **T, X, Y and Z** on every cycle (selection mask
`0x0F`); the client cannot select fewer channels through these STM interfaces.
They all use the board-configured, active-high DRDY input, issue `RM` on
completion, convert valid magnetic results to nT and temperature to mC, and
optionally generate the same formatted text report. A timestamp, hardware
sample sequence number, and lossless capture guarantee are **not** provided.

### Selecting and starting an STM

The target application's `sysconf.h` must select one mode. The low-level
identifier `MLX90393_INTR_MODE` is provided by the application; define the
other two identifiers with distinct values that do not conflict with the
project's existing mode values. The examples below use relative values.
Only the selected STM's implementation is active, even though the build
includes all three source files.

Periodic Burst (`sysconf.h`):

```c
#define MLX90393_SENSOR_MODE      MLX90393_INTR_MODE
#define MLX90393_BURST_RATE_HZ    10U
#define MLX90393_HARDIRON_EN      0U
```

```c
#include "mlx90393_stm.h"

/* Called once after the board HAL and FreeRTOS facilities are initialized. */
init_mlx90393_stm();
```

Periodic Single (`sysconf.h`):

```c
#define MLX90393_SINGLE_MODE      (MLX90393_INTR_MODE + 1)
#define MLX90393_SENSOR_MODE      MLX90393_SINGLE_MODE
#define MLX90393_MEASURE_PERIOD_MS 10U
#define MLX90393_HARDIRON_EN      0U
```

```c
#include "mlx90393_stm_single.h"

init_mlx90393_stm_single();
```

Continuous Burst (`sysconf.h`):

```c
#define MLX90393_CONT_MODE        (MLX90393_INTR_MODE + 2)
#define MLX90393_SENSOR_MODE      MLX90393_CONT_MODE
#define MLX90393_HARDIRON_EN      0U
```

```c
#include "mlx90393_stm_cont.h"

init_mlx90393_stm_cont();
```

For all three modes, configure `MLX90393_TASK_STACK_SIZE` and
`MLX90393_TASK_PRIO` in the target project. The STM initializes SPI in
mode 3 at 10 MHz, creates its binary semaphore, installs the `mlxs`
diagnostic command and creates a system-lifetime acquisition task. It
performs soft reset, communication probe and sensor configuration inside
that task; the initialization function does **not** wait for the first sample.
The board must provide the configured SPI, GPIO, DRDY interrupt and FreeRTOS
services. The STM is not a runtime-selectable or multi-instance interface.
Changing its compile-time settings requires rebuilding the firmware.

### Receiving measurements

The STM offers three independent optional output paths configured in `sysconf.h`:

- `MLX90393_REPORT_SI=1`: send each formatted sample to the project message
  logger (`msg(INF, ...)`). This is useful for tests but may limit throughput.
- `MLX90393_REPORT_FN=1`: build the report callback API
  `mlx90393_stm_set_report_fn(boolean_t (*)(const char *))`. The callback
  receives each formatted sample after a successful `RM` read. The same
  function name is supplied by the **selected** STM only.
- `MLX90393_REPORT_SAMPLE_FN=1`: build the binary sample callback API
  `mlx90393_stm_set_sample_fn(boolean_t (*)(const struct mlx90393_sample *))`.
  Each sample contains unchanged sensor RAW X/Y/Z/T, their validity flags,
  processed X/Y/Z in nT and converted temperature in mC.

All three output paths may be enabled at once. With all disabled, acquisition
still runs without exposing samples to the application. A callback receives
no data until registered; passing `NULL` unregisters it. Register during
startup, ideally before creating the STM task:

```c
/* Example for the selected Single STM, with MLX90393_REPORT_FN=1. */
#include "mlx90393_stm_single.h"

static boolean_t sample_report(const char *line)
{
	/* Copy line to application-owned storage if it must be retained. */
	(void) line;
	return (TRUE);
}

void start_magnetometer(void)
{
	mlx90393_stm_set_report_fn(sample_report);
	init_mlx90393_stm_single();
}
```

The callback is called **synchronously in the acquisition task**, not from
an ISR. The string is NUL-terminated, includes a newline, belongs to the
STM and is reused on the next report. Do not retain its pointer. Avoid
blocking I/O or lengthy processing in the callback, especially in
Continuous Burst mode. Returning `FALSE` increments `report_fn_err` in
`mlxs` but does not stop measurement or reset the STM. Callback registration
is not synchronized with concurrent reporting.

For the binary callback, define `MLX90393_REPORT_SAMPLE_FN=1` and use the
same registration function with every STM mode. The structure is declared
in `mlx90393.h`:

```c
struct mlx90393_sample {
	struct mlx90393_measure_data raw;
	int32_t x_nt;
	int32_t y_nt;
	int32_t z_nt;
	int32_t t_mc;
};
```

The STM fills the structure after a successful `RM`. Magnetic nT values
have already undergone any configured software hard-iron correction and
`REPORT_FLIP_X`, just like formatted text output. Temperature is converted
to mC without magnetic corrections. `raw` is always unchanged: with
`TCMP_EN=0` magnetic words use signed two's-complement coding; with
`TCMP_EN=1` they are unsigned with a zero at `0x8000` for the STM's
current configuration. A converted value of `INT32_MIN` means invalid or
unrepresentable; check `raw.x_valid`, `.y_valid`, `.z_valid` and `.t_valid`
for channel validity. A valid RAW axis with `INT32_MIN` in its nT field
indicates hard-iron correction overflow.

```c
/* Single STM example: create mag_queue during startup with item size
 * sizeof(struct mlx90393_sample). */
#include <queue.h>
#include "mlx90393_stm_single.h"

extern QueueHandle_t mag_queue;

/**
 * sample_report
 */
static boolean_t sample_report(const struct mlx90393_sample *sample)
{
	return (xQueueSend(mag_queue, sample, 0) == pdPASS);
}

/**
 * start_magnetometer
 */
void start_magnetometer(void)
{
	mlx90393_stm_set_sample_fn(sample_report);
	init_mlx90393_stm_single();
}
```

The binary callback executes synchronously in the acquisition task, not
in an ISR. Its pointer is only valid during the call: copy the structure
or use a nonblocking queue send. A `FALSE` return increments
`sample_fn_err` in `mlxs` without resetting the STM. No lossless capture
is guaranteed, especially in Continuous Burst mode; slow callbacks can
cause lost conversions or missed scheduled periods. Binary samples do
not contain a timestamp or a sequence counter.

With `MLX90393_REPORT_MSG_NUM=1`, a report looks like:

```text
MLX90393 si [042]: X=+6.594 Y=+27.475 Z=-1.012 uT T=+17.124 C
```

With that option disabled, the `[042]` field is omitted. The index wraps
from `999` to `000`; it counts formatted reports, **not** sensor conversions.
`MLX90393_REPORT_FLIP_X=1` changes the sign of reported X after any
hard-iron correction. X/Y/Z are in microtesla and T is in degrees Celsius,
all with three decimal places; an invalid channel is displayed as
`----.---`. The underlying conversion uses integer nT/mC, so three decimal
places express the report resolution, **not** guaranteed sensor accuracy.
No hardware timestamp or raw 16-bit measurement is included in the string.

### Build-time configuration reference (`sysconf.h`)

The following settings apply to **all three** STM implementations:

| Setting | Allowed values / default | Effect on client data |
| --- | --- | --- |
| `MLX90393_RANGE_PROFILE` | `MLX90393_RANGE_5MT` (default), `_25MT`, `_50MT` | Selects XY digital range preset (`GAIN_SEL`, `RES_X/Y/Z`); the Z scale differs. Does not change physical sensor saturation. |
| `MLX90393_OSR` | 0..3; default 0 | Magnetic ADC oversampling; affects noise and conversion time. |
| `MLX90393_DIG_FILT` | 0..7; default 4 | Magnetic digital filter; affects noise and conversion time. |
| `MLX90393_OSR2` | 0..3; default 0 | Temperature oversampling; affects conversion time. |
| `MLX90393_TCMP_EN` | 0 or 1; default 0 | Enables on-chip temperature compensation of magnetic sensitivity. |
| `MLX90393_HARDIRON_EN` | **Required**, 0 or 1 | Enables fixed software per-axis offset subtraction. |
| `MLX90393_HARDIRON_X_NT`, `_Y_NT`, `_Z_NT` | Signed int32 nT; required only when hard-iron is enabled | Independent fixed offsets in native sensor axes. |
| `MLX90393_REPORT_SI` | 0 or 1 | Enable formatted message logging. |
| `MLX90393_REPORT_FN` | 0 or 1 | Enable formatted report callback. |
| `MLX90393_REPORT_SAMPLE_FN` | 0 or 1; default 0 if undefined | Enable RAW plus processed SI sample callback. |
| `MLX90393_REPORT_FLIP_X` | 0 or 1 | Invert the reported X axis after offset correction. |
| `MLX90393_REPORT_MSG_NUM` | 0 or 1 | Include rolling `[000]`..`[999]` report index. |
| `MLX90393_TASK_STACK_SIZE`, `MLX90393_TASK_PRIO` | Project-defined | Acquisition task resources and scheduling priority. |

With `HALLCONF=0xC`, the combinations `(OSR,DIG_FILT)=(0,0), (0,1),
(1,0)` are rejected at compile time. Other out-of-range fields are also
rejected. Increasing oversampling/filtering generally increases conversion
time and may reduce achievable rate. For the default 5 mT profile with
`TCMP_EN=0`, typical magnetic conversion factors are 150 nT/LSB for X/Y
and 242 nT/LSB for Z; with `TCMP_EN=1` they are 157 and 253 nT/LSB.
The range presets refer to digital XY scaling, not a guarantee that the
sensor remains unsaturated at the named field strength.

Mode-specific controls are deliberately separate:

| Setting | Periodic Burst | Periodic Single | Continuous Burst |
| --- | --- | --- | --- |
| `MLX90393_BURST_RATE_HZ` | Used; defaults to 10 Hz request | Not used | Not used |
| `MLX90393_MEASURE_PERIOD_MS` | Not used | **Required**; 1..1000 ms, subject to timing/tick checks | Not used |
| BDR programmed by STM | Computed, 1..63 | 0, not used for SM scheduling | 0, enables continuous burst after SB |
| Sample channel selection | Fixed TXYZ | Fixed TXYZ | Fixed TXYZ |

The settings are compiled into the selected STM; there is no run-time
sample-rate setter or mode-switching API.

### Periodic Burst: sensor-paced sampling (`MLX90393_INTR_MODE`)

The STM sends one `SB` command after configuration. The MLX90393 then
performs successive TXYZ burst conversions with an idle interval
selected by `BDR=1..63`; each completed conversion produces DRDY and an
interrupt-driven `RM` read. The MCU does not schedule individual bursts.

`MLX90393_BURST_RATE_HZ` is a **whole-Hz nominal upper target**, not an
exact output frequency. The STM estimates conversion time from OSR,
DIG_FILT and OSR2, then rounds the idle interval **up to 20 ms steps**.
At the default TXYZ settings, the estimated conversion time per burst
is 4376 us and the total nominal period is:

```text
T_BURST = 4376 us + BDR * 20000 us      (BDR=1..63)
```

Representative settings with default oversampling/filtering:

| Requested rate | BDR | Nominal delivered rate |
| ---: | ---: | ---: |
| 1 Hz | 50 | ~0.996 Hz |
| 5 Hz | 10 | ~4.893 Hz |
| 10 Hz | 5 | ~9.581 Hz |
| 25 Hz | 2 | ~22.535 Hz |
| 42 Hz | 1 | ~41.024 Hz |

For those defaults, requests 1..42 Hz are accepted. Other filter/OSR
settings change both the allowed range and the mapping. The full
request-to-BDR table appears in `mlx90393_stm.h`. The actual rate may
differ from the nominal one because of sensor oscillator tolerance.
This mode is suitable when the sensor should run autonomously at a
moderate rate without FreeRTOS timing each conversion.

### Periodic Single: MCU-paced sampling (`MLX90393_SINGLE_MODE`)

The STM issues one `SM` command per scheduled period and waits for DRDY.
It reads TXYZ via `RM` and returns to its FreeRTOS period schedule. The
sensor does **not** run a burst sequence between commands. BDR is set to
zero in CFG1 but does not turn this SM-only acquisition into Continuous
Burst, because no `SB` command is issued.

`MLX90393_MEASURE_PERIOD_MS` specifies the intended time **between SM
starts**, not a guaranteed report-delivery interval. The STM uses
`vTaskDelayUntil()` to keep scheduled starts aligned with FreeRTOS ticks.
The selected period must be representable as an exact whole number of
ticks and must exceed the nominal conversion time plus the 140 us
post-DRDY delay. At the default settings the nominal SM conversion time
is 4596 us; `10U` requests 100 measurements/s. A valid build-time
period is not a guarantee that SPI, reporting and other tasks will
always meet the deadline.

If a cycle runs late, the STM skips elapsed schedule slots; it does not
queue measurements or issue back-to-back SM commands to catch up. The
`missed_period` diagnostic tracks skipped slots. Use this mode for a
client-specified sample schedule, with explicit awareness of FreeRTOS
tick resolution, ISR latency and downstream processing load.

### Continuous Burst: sensor-paced maximum rate (`MLX90393_CONT_MODE`)

The STM programs `BDR=0` and sends **one** `SB` command. The sensor then
converts TXYZ continuously, without a 20 ms idle interval. Each DRDY
causes an interrupt-driven `RM` read. The MCU does not issue periodic
`SM` commands, and neither `MLX90393_BURST_RATE_HZ` nor
`MLX90393_MEASURE_PERIOD_MS` affects this mode.

The nominal conversion-to-conversion interval is:

```text
T_CONVM = 67 + 64 * 2^OSR * (2 + 2^DIG_FILT)    us
T_CONVT = 67 + 192 * 2^OSR2                       us
T_CONT  = 3*T_CONVM + T_CONVT + 100                us
```

With `OSR=0`, `DIG_FILT=4`, `OSR2=0`, TXYZ: `T_CONT=4016 us`, or
approximately **249 samples/s nominal**. Increasing the conversion
time reduces this sensor-defined frequency. The true rate depends on
the sensor oscillator and how quickly the MCU consumes samples. The
initial conversion after `SB` can have additional startup overhead.

There is no sensor FIFO: if another conversion completes while the
previous data is still pending, an intermediate result may be replaced.
The STM handles the brief DRDY LOW strobe associated with overlapping
conversions and reports successful `RM` reads, but cannot infer the exact
number of lost sensor conversions. Disable per-sample serial logging
and keep callbacks short if maximum throughput matters.

### Temperature compensation and corrections

All three modes accept `MLX90393_TCMP_EN=0/1`:

- `0` (default): original signed magnetic RAW coding at the 25 C
  sensitivity reference. The STM does not write the sensor OFFSET registers.
- `1`: on-chip magnetic **sensitivity** temperature compensation using
  factory-trimmed coefficients. The STM writes neutral `0x8000` values
  to OFFSET_X/Y/Z before enabling TCMP, and the LL conversion switches
  to unsigned 0x8000-centered coding and the 35 C reference sensitivity.

All modes already measure temperature, so enabling TCMP does not add an
extra temperature channel or change their nominal timing. Compensation
does not remove hard-iron field bias, calibrate individual axes, eliminate
instantaneous noise, or guarantee temperature-independent results.

`MLX90393_HARDIRON_EN=1` independently subtracts the configured signed
X/Y/Z offsets **in nT** from each converted magnetic sample. A subsequent
`MLX90393_REPORT_FLIP_X=1` reverses only the X sign. Hardware OFFSET
register values used for TCMP are **neutral** and are not the software
hard-iron offsets. Temperature and LL RAW values are unaffected by the
software hard-iron correction.

### Data continuity, diagnostics and recovery

All STM readouts are interrupt driven. After DRDY the acquisition task
waits 140 us before reading. A timeout, SPI/status error or other fatal
STM state error triggers a **5-second delay**, followed by a sensor reset,
probe, reconfiguration and restart of acquisition. Application code
should not infer that a configured STM necessarily delivers every
sample indefinitely. No application-facing sample backlog is provided.

The terminal command `mlxs` exposes counters for the active STM:

| Mode | Counters | Interpretation |
| --- | --- | --- |
| Periodic Burst | `read_data_reg`, `wait_int_tmo`, `intr_pin_low`, `stm_rst` | Successful RM reads; interrupt timeouts; unexpected DRDY LOW; STM resets. |
| Periodic Single | `start`, `read`, `missed_period`, `wait_int_tmo`, `intr_pin_low`, `stm_rst` | SM commands; successful RM reads; skipped schedule slots; interrupt timeouts; unexpected DRDY level; STM resets. |
| Continuous Burst | `read`, `wait_int_tmo`, `drdy_low`, `stm_rst` | Successful RM reads; interrupt timeouts; observed LOW strobes; STM resets. |

For Periodic Single, the legacy `intr_pin_low` field counts two unexpected
DRDY states: HIGH before issuing `SM` (expected LOW), or LOW after the
interrupt wakeup (expected HIGH). The name is retained for compatibility.

When text callback reporting is built in, `report_fn_err` is displayed:
it counts callbacks returning `FALSE`, not missing sensor conversions.
With the binary callback enabled, `sample_fn_err` similarly counts its
`FALSE` returns and is shown independently.

The Continuous Burst `drdy_low` counter is not, by itself, a fatal error;
a brief LOW strobe may occur while a new conversion replaces an older
result. A zero error count and continuous report indices do not prove
that all internal sensor conversions were captured. For rate validation,
use an independent time reference or hardware observation of DRDY.

For implementation details, precise timing assumptions and register setup,
see `mlx90393_stm.h`, `mlx90393_stm_single.h` and
`mlx90393_stm_cont.h`. The direct low-level access API is documented in
`mlx90393.h`.

## License

See the license information in the source files.

## Author

Jan Rusnak  
AZTech
