# Wolklamp V2

This is the repository for Wolklamp V2, a feature-rich, ESP32-S3 based smart lighting controller. It is designed to run on a dual-core ESP32, with one core dedicated to networking and the other to real-time light and input management.

## Project Overview
Wolklamp V2 provides simultaneous and independent control over three distinct types of LED lighting, making it a versatile solution for complex lighting installations. It is fully integrated with Home Assistant via MQTT Discovery and can also be controlled by a physical power switch.

## Features

*   **Triple Light Control:**
    *   **"Wolk Kleur" (Cloud Color):** Manages a 120-pixel SK6812 RGBW addressable LED strip.
    *   **"Wolk Wit" (Cloud White):** Controls a single-channel white LED strip via a MOSFET with high-frequency PWM.
    *   **"Sterrenhemel" (Starry Sky):** Drives 24 individual LEDs using a TLC5947 driver to create a twinkling star effect with randomized patterns.
*   **Advanced Lighting Effects:**
    *   Smooth, configurable fading for all light sources.
    *   Perceptually linear brightness through Gamma Correction.
    *   "Smart Power Limiting" on the RGBW strip to prevent power supply overload.
*   **Smart Home Integration:**
    *   **WiFiManager:** Provides a web portal for easy initial Wi-Fi setup.
    *   **MQTT Control:** Full control and state reporting over MQTT.
    *   **Home Assistant Discovery:** Automatically registers all three light entities and additional configuration entities in Home Assistant.
*   **Dual Control Schemes:**
    *   Full remote control via MQTT for smart home integration.
    *   Physical control via a standard 230V power switch:
        *   A short power cycle toggles between the three light sources.
        *   A long power cycle (off for >2 seconds) resets to a default state (turning on the "Wolk Wit" strip).
*   **Developer Friendly:**
    *   **Over-the-Air (OTA) Updates:** Allows for wireless firmware updates without physical access.
    *   **Telnet Logging:** Provides a remote stream for debugging.
    *   **FreeRTOS based:** Uses a multi-core architecture for robust, non-blocking performance.

## Hardware Components
The firmware is built for the following key components:
*   **Microcontroller:** ESP32-S3
*   **RGBW Strip:** SK6812 Addressable LEDs (e.g., NeoPixel)
*   **Starry Sky Driver:** Adafruit TLC5947 (for 24-channel PWM)
*   **White Strip Driver:** A standard N-Channel MOSFET

## Setup
To set up the project, you will need:
*   [PlatformIO IDE](https://platformio.org/) (VSCode extension recommended)
*   An ESP32-S3 development board and the other hardware components listed above.

Clone the repository:
```bash
git clone https://github.com/RSelten/WolklampV2.git
cd WolklampV2/Wolklamp
```

Open the project in PlatformIO. You will need to configure your MQTT broker details in `src/main.cpp`. After configuration, build and upload the firmware to your ESP32 board.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
