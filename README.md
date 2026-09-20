# ADIS16470-Sensor
This project implements a driver for the ADIS16470 Inertial Measurement Unit (IMU) sensor, which communicates with a microcontroller via UART. The driver initializes the IMU, configures it to output data at a specified frequency, and processes the received data to extract orientation information (yaw, pitch, roll). The processed data is then sent to a PC terminal for display.

The main functionalities of the driver include:
- Initializing the IMU and setting up UART communication.
- Starting and stopping the IMU data output.
- Handling UART receive interrupts to capture incoming data.
- Parsing the received data to extract raw sensor readings.
- Calculating the orientation angles (yaw, pitch, roll) from the raw sensor data.

The driver is designed to be used in embedded systems where real-time orientation data from the IMU is required, such as in robotics, drones, or motion tracking applications.

## Acknowledgements
* [DM-MC02](./doc/达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf)
* [ADIS16470 Datasheet](./doc/ADIS16470.pdf)