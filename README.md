
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

## Fusion Approach

Combines:
- IMU integration (fast, drift-prone)
- Barometric differentiation (stable, slower)

## Hardware

- STM32 (F401 series)
- MPU6050
- BMP280 x2

## Use Cases

UAVs, gliders, and embedded flight instrumentation.
