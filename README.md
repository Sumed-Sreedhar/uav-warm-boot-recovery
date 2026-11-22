# uav-warm-boot-recovery
Deterministic warm-boot fast recovery for UAV flight computers on STM32F446RE. A dual-path bootloader tells cold power-on from watchdog resets, skips IMU calibration on warm boots, and restores FreeRTOS flight tasks in milliseconds with CRC-validated state.
