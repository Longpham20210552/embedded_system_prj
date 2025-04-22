# Embedded Humidity Monitoring System

This embedded system project periodically acquires air humidity data and displays it via a serial interface or LCD. It demonstrates both **single-task** and **multi-task** scheduling techniques to manage sensor reading, data logging, and display operations.

## Features

- Periodic **humidity sensor reading**
- **Data display** via UART serial or LCD (I2C)
- **Data logging** using internal/external storage (optional)
- Implementation of various **task scheduling techniques**

## Hardware Specification

Target platform: **STM32F103C8T6 (Blue Pill)**  
Clock: 72 MHz, 20 KB RAM, 64 KB Flash

### Connected Modules

| Module               | Interface | STM32 Pins Used | Description                                 |
|----------------------|-----------|------------------|---------------------------------------------|
| **DHT11**            | 1-Wire    | `PB9`           | Reads temperature and humidity              |
| **LCD1602 (I2C)**    | I2C       | `PB6` (SCL), `PB7` (SDA) | Displays sensor data (via I2C backpack) |
| **DS1307 RTC**       | I2C       | `PB6` (SCL), `PB7` (SDA) | Real-time clock module                      |
| **CP2102 UART-USB**  | UART      | `PA9` (TX), `PA10` (RX) | Serial communication with PC               |

## Scheduling Techniques

### Single-task Scheduling

Two main strategies using blocking loop:
- `main_loop`: Sensor read → log → display (sequential)
- `time_slice`: Split time for each task inside main loop

### Multitasking Scheduling

- `Round Robin`: Tasks executed cyclically without priority
- `Rate Monotonic Scheduling (RMS)`: Frequent tasks get higher priority
- **Implemented using**:
  - **RTOS** (e.g., FreeRTOS), or
  - **Timer Interrupts** for cooperative multitasking

### Final product demo
![image](https://github.com/user-attachments/assets/dff5e899-4dc0-4bb3-9afb-16d05c780526)
