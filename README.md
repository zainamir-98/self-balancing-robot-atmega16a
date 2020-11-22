# Self-Balancing Robot using an AtMega16A

This is a project I worked on for my Microprocessor Systems course.

Equipment
* AtMega16A 8-bit microcontroller (x1)
* MPU6050 accelerometer and gyroscope sensor (x1)
* L298N motor driver modules (x2)
* Rechargeable batteries (x3)

The MPU6050 was connected with the AtMega16A through I2C. A PID control library was used to implement the control system.

Link to I2C library: https://github.com/Sovichea/avr-i2c-library

The PID library is included in this repository (courtesy of Davide Gironi).
