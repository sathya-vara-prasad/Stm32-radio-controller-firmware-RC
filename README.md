#STM32 RC Controller Firmware
----------------------------------------------
    A multi-channel RC transmitter firmware for STM32 supporting USB HID Joystick and SBUS output, featuring DMA-based ADC sampling, median + EMA filtering, and per-channel calibration.

#Features
-----------------------------------------------
    --> USB HID Joystick — plug-and-play with simulators, games, and GCS software

    --> SBUS Output — 16-channel, 11-bit resolution over UART (100k baud, inverted)

    --> 6-channel ADC input — 4 joystick axes + 2 potentiometers

    --> Digital inputs — 2× 3-way switches, 2× toggle switches, 4× push buttons

    --> Median + EMA filtering for spike removal and smooth response

    --> Deadzone & center calibration with auto-detect at startup

    --> DMA-based continuous ADC sampling for low-latency control loop

# System Architecture
--------------------------------------------------
text
ADC (DMA) → Median Filter → EMA Filter → Calibration → Mapping
                                                  ├── USB HID Joystick
                                                  └── SBUS Encoder → UART TX

# Input Mapping
--------------------------------
Joystick Axes
Axis	Channel	Function
X	CH4	Roll
Y	CH3	Pitch
Z	CH1	Throttle
RZ	CH2	Yaw
Potentiometers
Input	Channel
POT1	CH5
POT2	CH6
Switches & Buttons
Input	Channel
3-way switch × 2	CH7, CH8
Toggle switch × 2	CH9
Push button × 4	CH10 – CH11
📡 SBUS Protocol
Parameter	Value
Baud Rate	100,000
Frame Size	25 bytes
Channels	16 (11-bit each)
Range	172 – 1811
Logic	Inverted UART

# Hardware Requirements
----------------------------------------
    STM32F1 series (or compatible)

    2× Analog joystick modules

    2× Potentiometers

    2× 3-way switches, 2× toggle switches, 4× push buttons

    USB connection (for HID mode)

    SBUS-compatible flight controller

# Project Structure
---------------------------------------
text
├── Core/
│   ├── Src/
│   │   ├── main.c
│   │   └── usb_device.c
│   └── Inc/
├── USB_DEVICE/
└── Drivers/

# Build & Flash
------------------------------------------
    Open the project in STM32CubeIDE

    Build the project (Ctrl+B)

    Flash to your STM32 board

    Connect:

        USB → PC (registers as HID joystick)

        UART TX → Flight controller SBUS input

# Debugging
-----------------------------------------
    Optional UART debug print output

    Onboard LED toggles to indicate control loop activity

# Roadmap
--------------------------------------
    EEPROM / Flash calibration storage

    RF transmission integration (SX1280 / FLRC / LoRa)

    Video + telemetry integration

    Kalman filter for advanced smoothing

👨‍💻 Author
----------------------------------------
Sathya — Embedded Systems Developer

If you find this project useful, please consider giving it a ⭐ on GitHub!
