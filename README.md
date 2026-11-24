# UAV Warm-Boot Recovery

Embedded C firmware for a UAV flight computer on the **STM32F446RE**. The project investigates how quickly the flight computer can resume valid control after a watchdog reset, and when a saved attitude estimate is too stale to reuse. A separate Python simulator will provide sensor data and ground truth for closed-loop hardware-in-the-loop (HIL) tests.

## Current status

The standalone flight application builds with STM32CubeIDE and has been tested on a **NUCLEO-F446RE**. The board has produced both startup messages, blinked LD2 from a FreeRTOS task, and echoed a short message through USART1 DMA at 921600 baud. Sustained traffic across DMA buffer wraps is the next bench check. HIL packet parsing, estimation, PID, checkpointing, and warm recovery are planned.

## Build and flash

1. In STM32CubeIDE, use **File → Import → Existing Projects into Workspace** and select the [`flight-app/`](flight-app/) directory.
2. Build the **Debug** configuration with CubeIDE's bundled GNU Tools for STM32.
3. Connect the Nucleo over its ST-LINK USB port and run or debug the **flight-app** project to flash it. If the debugger stops at `main()`, resume execution.
4. Use [`flight-app.ioc`](flight-app/flight-app.ioc) to inspect or regenerate the CubeMX configuration. Keep hand-written edits within the generated user-code sections or in separate source files.

## UART bring-up

| Interface | Connection | Serial settings | Expected behavior |
| --- | --- | --- | --- |
| USART2 debug | ST-LINK virtual COM port | 115200 baud, 8N1, no flow control | `flight-app boot` once per reset and any USART1 loopback errors |
| USART1 HIL transport | External 3.3 V USB-to-UART adapter | 921600 baud, 8N1, no flow control | `USART1 hello test` once per reset, then an exact echo of received bytes |

Use three wires for the external adapter. The Nucleo stays powered through ST-LINK, and the adapter's VCC pin stays disconnected.

| USB-to-UART adapter | NUCLEO-F446RE |
| --- | --- |
| TXD | D2 (PA10, USART1 RX) |
| RXD | D8 (PA9, USART1 TX) |
| GND | GND |

On Linux, find the two devices with `ls -l /dev/serial/by-id/`. Open each port in its own terminal. Replace the device names below with the ones reported on your machine.

```sh
picocom -b 115200 /dev/ttyACM0  # ST-LINK
picocom -b 921600 /dev/ttyUSB0  # External adapter, in a second terminal
```

Open both ports before pressing **RESET** so you can see both startup lines. LD2 should complete one blink cycle per second. With local echo off in Picocom, type `hello` in the USART1 terminal. It should appear once. Exit Picocom with **Ctrl+A**, then **Ctrl+X**.

For the buffer-wrap test, send at least **513 known bytes** in one burst, then a multi-kilobyte sequence while reading the echo concurrently. Compare the returned length and contents byte for byte. [`hil_link.c`](flight-app/Core/Src/hil_link.c) uses a 512-byte circular RX DMA buffer and a separate normal-mode TX DMA buffer. On a detected UART/DMA error or receive overrun, the loopback reports the failure on USART2 and stops until reset.

## Planned flight architecture

The Python simulator sends raw gyro, accelerometer, and magnetometer samples over USART1. The MCU parses those frames, and a periodic FreeRTOS task reads the latest fresh sample, updates the attitude estimate, runs PID, and returns the control output. The simulator continues advancing during a reset and records ground truth. All sensor inputs for this experiment come from the simulator.

- **Checkpointing.** Commit state late in each control iteration to one of two slots in a fixed `.noinit` region in SRAM1. Store the attitude quaternion, gyro bias, arming state, GPS lock validity, sequence ID, sentinel, and CRC32. Keep DMA buffers outside the retained region.
- **Warm recovery.** A watchdog reset triggers a recovery attempt. Check checkpoint integrity, then use a conservative reset-gap bound and fresh sensor observations to decide whether to accept, correct, or reject the attitude. Evaluate gyro-bias validity separately. If attitude cannot be reused, attempt sensor-based reinitialization from usable accelerometer and magnetometer observations without requiring stationary calibration in flight.
- **Evaluation.** Compare cold boot, blind warm restore, sensor-based warm recovery, and the proposed check. Inject IWDG starvation, WWDG deadline overruns, and software resets during checkpoint writes. Measure watchdog detection separately from reset-to-first-valid-control-output recovery. Use DWT cycle counts and GPIO scope markers for timing, and simulator ground truth for attitude error and false-accept and false-reject rates.

The build order is flight application and HIL first, then application-level warm recovery, then the bootloader and final timing measurements. Brownout, main-power loss, retained FreeRTOS task stacks, and bootloader MPU setup are out of scope.
