# STM32 RC Controller Firmware

> A multi-channel RC transmitter firmware for STM32 supporting USB HID Joystick and SBUS output,
> featuring DMA-based ADC sampling, median + EMA filtering, and per-channel calibration.

![Platform](https://img.shields.io/badge/Platform-STM32F1-blue)
![Language](https://img.shields.io/badge/Language-C-lightgrey)
![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE-green)
![License](https://img.shields.io/badge/License-MIT-yellow)

---

## Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Input Mapping](#input-mapping)
- [SBUS Protocol](#sbus-protocol)
- [Hardware Requirements](#hardware-requirements)
- [Project Structure](#project-structure)
- [Build & Flash](#build--flash)
- [Debugging](#debugging)
- [Roadmap](#roadmap)
- [Author](#author)

---

## Features

| Feature | Description |
|---|---|
| 🎮 USB HID Joystick | Plug-and-play with simulators, games, and GCS software |
| 📡 SBUS Output | 16-channel, 11-bit resolution over UART (100k baud, inverted) |
| 🎛️ 6-Channel ADC | 4 joystick axes + 2 potentiometers via DMA circular sampling |
| 🔘 Digital Inputs | 2× 3-way switches, 2× toggle switches, 4× push buttons |
| 🧹 Dual Filtering | Median filter (spike removal) + EMA filter (smooth response) |
| 📐 Calibration | Auto center detection at startup with configurable deadzone |
| ⚡ DMA Sampling | Continuous circular ADC acquisition for low-latency control loop |

---

## System Architecture

```text
ADC (DMA) → Median Filter → EMA Filter → Calibration → Mapping
                                                  ├── USB HID Joystick
                                                  └── SBUS Encoder → UART TX
```

---

## Input Mapping

### Joystick Axes

| Axis | Channel | Function |
|------|---------|----------|
| X    | CH4     | Roll     |
| Y    | CH3     | Pitch    |
| Z    | CH1     | Throttle |
| RZ   | CH2     | Yaw      |

### Potentiometers

| Input | Channel |
|-------|---------|
| POT1  | CH5     |
| POT2  | CH6     |

### Switches & Buttons

| Input             | Channel     |
|-------------------|-------------|
| 3-way switch × 2  | CH7, CH8    |
| Toggle switch × 2 | CH9         |
| Push button × 4   | CH10 – CH11 |

---

## SBUS Protocol

| Parameter  | Value            |
|------------|------------------|
| Baud Rate  | 100,000          |
| Frame Size | 25 bytes         |
| Channels   | 16 (11-bit each) |
| Range      | 172 – 1811       |
| Logic      | Inverted UART    |

---

## Hardware Requirements

- STM32F1 series microcontroller (or compatible)
- 2× Analog joystick modules
- 2× Potentiometers
- 2× 3-way switches, 2× toggle switches, 4× push buttons
- USB connection (for HID mode)
- SBUS-compatible flight controller

---

## Project Structure

├── Core/
│ ├── Src/
│ │ ├── main.c
│ │ └── usb_device.c
│ └── Inc/
├── USB_DEVICE/
└── Drivers/


---

## Build & Flash

1. Open the project in **STM32CubeIDE**
2. Build — `Project → Build All` or `Ctrl+B`
3. Flash to your STM32 board via ST-Link
4. Connect peripherals:
   - **USB** → PC (registers as HID joystick automatically)
   - **UART TX** → Flight controller SBUS input

---

## Debugging

- Optional UART debug output for live ADC values and channel data
- Onboard LED toggles on every control loop cycle to confirm activity

---

## Roadmap

- [ ] EEPROM / Flash-based calibration storage
- [ ] RF transmission integration (SX1280 / FLRC / LoRa)
- [ ] Video + telemetry pipeline integration
- [ ] Kalman filter for advanced input smoothing

---

## Author

**Sathya** — Embedded Systems Developer

> Built with for the drone and RC community.
> If you found this project helpful, please consider giving it a ⭐ on GitHub!
