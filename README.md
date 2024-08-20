# BLEServer

## Overview

This project demonstrates communication between two ESP32 devices using Bluetooth Low Energy (BLE) with the BLEServer library. The setup involves a server-client architecture where data is exchanged seamlessly between the two devices.

## Features

- **UUID Characteristics**: Utilizes UUID characteristics for sending and receiving data.
- **Potentiometer Data Processing**: The server reads and processes potentiometer data.
- **PWM LED Control**: The client uses the received data to control an LED with Pulse Width Modulation (PWM).

## Detailed Description

### Server Side
- Reads potentiometer data.
- Processes the data.
- Sends the processed data to the client via BLE.

### Client Side
- Receives data from the server.
- Uses the data to control an LED with PWM.

## Diagram

![Pictoric BLEServer Diagram](pictoric.jpg)

## Getting Started

1. **Hardware Requirements**:
   - Two ESP32 devices.
   - A potentiometer.
   - An LED.

2. **Software Requirements**:
   - Arduino IDE or PlatformIO.
   - BLEServer library.

3. **Setup**:
   - Connect the potentiometer to the server ESP32.
   - Connect the LED to the client ESP32.
   - Upload the respective server and client code to the ESP32 devices.

4. **Running the Project**:
   - Power on both ESP32 devices.
   - The server will start reading potentiometer data and send it to the client.
   - The client will receive the data and adjust the LED brightness accordingly.

## Documentation Resource

For more detailed information on BLE with ESP32, refer to the [ESP32 BLE UART Documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ble.html#ble-uart).

## Last Thoughts

This project showcases the capabilities of BLE communication between ESP32 devices, enabling real-time data exchange and control. It serves as a foundation for more complex IoT applications.

Feel free to explore and modify the code to suit your specific needs!