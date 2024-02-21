# LiDAR Lite v3 Library for Raspberry Pi in Python

This library provides quick access to basic functions of the LiDAR Lite v3 via I2C on a Raspberry Pi. It uses Garmin's [official repository for the Lidar Lite on Arduino](https://github.com/garmin/LIDARLite_Arduino_Library/) as a template.

## Installation Instructions
- Ensure Lidar Lite v3 is connected to I2C bus 1 address 0x62 via `i2cdetect -y 1`.
    - If necessary, update the I2C address class instance.
    - Alternatively, provide the correct address when instantiating the Lidar class
- Download this repo and import the `Lidar` class from `LLV3.Lidar_Lite_V3`.
- `examples.py` provides samples for taking measurement individual and burst readings.