

<img width="879" height="1280" alt="WhatsApp Image 2026-04-24 at 5 04 05 PM" src="https://github.com/user-attachments/assets/d250b559-9461-4d2c-9c77-574fa7a8a40d" />

# RTOS-Based Variometer (STM32)

Real-time variometer built on STM32 using sensor fusion between IMU (MPU6050) and barometric sensors (BMP280) to estimate vertical velocity with low latency and high stability.

## Features

- ~1 ms loop using CMSIS-RTOS (FreeRTOS)
- Multi-threaded design: sensor, display, fault monitor
- Complementary fusion (IMU + barometer)
- Tilt compensation using Kalman-filtered roll/pitch
- Runtime bias estimation for drift reduction
- Dual BMP280 with automatic failover
- UART telemetry output (Vz, altitude, pitch)

<img width="657" height="771" alt="WhatsApp Image 2026-04-24 at 5 18 38 PM" src="https://github.com/user-attachments/assets/ae236368-7457-47dc-993e-5fe147c6d690" />

## Fusion Approach

Combines:
- IMU integration (fast, drift-prone)
- Barometric differentiation (stable, slower)

## Hardware

- STM32 (F401 series)
- MPU6050
- BMP280 x2

<img width="730" height="787" alt="WhatsApp Image 2026-04-24 at 5 19 26 PM" src="https://github.com/user-attachments/assets/eb7ed2c3-4efc-4fa9-97b5-1452de7bbf49" />


## Use Cases

UAVs, gliders, and embedded flight instrumentation.


