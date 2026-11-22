# UAV Warm-Boot Recovery

Deterministic warm-boot fast recovery for a UAV flight computer on **STM32F446RE**, tested in a closed-loop HIL setup.

A dual-path bootloader tells a cold power-on apart from a watchdog reset. On a warm boot it skips gyro-bias calibration, validates the saved flight state with a CRC, re-arms the sensor link and restarts the FreeRTOS tasks in milliseconds. If validation fails it falls back to a full cold boot.

> This repo is the **embedded C firmware only**. The drone dynamics and sensor models live in a separate custom Python engine.

## Why

A watchdog reset on a normal flight computer repeats the full cold start, including a 2-3 s gyro calibration, while the vehicle is airborne. The calibrated state was valid milliseconds before the fault, so most of that work is wasted. This project makes each reset cheap and checks the result against simulated ground truth.

## Architecture

```
 Laptop (Python engine)                         STM32F446RE
┌─────────────────────────┐  sensor frames   ┌──────────────────────────────┐
│ vehicle dynamics        │ ───────────────► │ USART RX (DMA) → parser      │
│ simulated IMU/mag/GPS   │   921600 baud    │ Mahony attitude → PID        │
│ ground-truth attitude   │                  │ bootloader + warm-boot state │
│ applies PID outputs     │ ◄─────────────── │ USART TX (DMA)               │
└─────────────────────────┘  PID output      └──────────────┬───────────────┘
                             frames                         │ GPIO toggle
                                                            ▼
                                                    logic analyser
```

- **No physical sensors on the STM32.** All sensor data arrives from the laptop over UART (USART + DMA, 921600 baud).
- **Closed loop through the MCU.** The simulated vehicle only moves in response to PID outputs the real firmware computed, so a reset has a physical consequence in the sim.
- **The sim runs on the wall clock**, so the vehicle keeps flying while the MCU is down and the recovery gap is visible.
- **Control loop timing (proposed):** the control task runs on its own fixed period and reads the latest sensor frame, so a laptop stall looks like stale data and not a hung task.

## Boot paths

| | Cold boot (POR / NRST) | Warm boot (IWDG / WWDG) |
|---|---|---|
| Reset cause | read RCC reset flags, then clear them | same |
| State | zeroed, fresh sentinel | sentinel + CRC32 checked |
| Peripherals | full init | critical only, **incl. USART + DMA** |
| Gyro bias | calibrated (2-3 s) | loaded from saved state |
| On failure | n/a | fall back to cold boot |

After a warm boot the UART must be re-initialised (peripherals reset with the MCU), its error flags cleared, and the parser resyncs on the next valid frame. Frames sent during the reset are lost.

## Persistent state

Fixed-address struct in `.noinit` SRAM1: Mahony quaternion, gyro bias, arming state, GPS lock validity, sequence ID, magic sentinel, CRC32. Written and re-hashed once per control-loop iteration, late in the loop body. DMA buffers stay outside it.

Watchdog resets never remove VDD, so plain SRAM is enough. Surviving real power loss is out of scope.

## Faults and measurement

- **Faults (software only):** IWDG starvation (hung task), WWDG deadline overrun (late task), reset during the state-write step (torn write).
- **Latency:** DWT cycle counter plus a GPIO toggle for scope cross-check.
- **Drift:** angular error between simulator ground truth and the firmware estimate across the recovery gap.
- **Also tracked:** CRC pass/fail rate, fallback-to-cold rate, flash/SRAM footprint, latency vs. clock frequency.

## Tasks

FreeRTOS on STM32 HAL. Control (estimator + PID + state write), UART parser, fault state machine, and possibly a logger. Fault injection runs from a test-only task.

## Out of scope

Brownout / power-loss recovery, preserving live FreeRTOS TCBs and stacks, MPU setup in the bootloader.

## TBD

Memory map, frame format, task priorities and periods, whether PID integrator state is persisted, build and flash steps, link to the Python engine repo, license.
