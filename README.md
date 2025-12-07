# hexapod-sar
a hexapod search and rescue (sar) robot


Sensor dan Aktuator yg dipakai
- 20 Ax18 - uart
- 2 emax servo -  pwm
- 5 ir sensor  - analog
- MPU6050 (IMU) - i2c
- Pixy2 Camera - uart (res = 316x208 -> middle = 158x104)

Microcontroller 

! STM32F103 (Main algorithm, sensor, camera) !

UART PINS
( Pilih 2 untuk sambung ke nano dan esp32 langsung, 1nya jst [5V, GND, RX, TX] )

- UART1_TX - PB6
- UART1_RX - PB7
- UART2_TX - PA2
- UART2_RX - PA3
- UART3_TX - PB10
- UART3_RX - PB11

I2C PINS [5v, gnd, scl, sda]
- i2C1_SCL - PB8
- i2c1_SDA - PB9

Analog pins [5v, gnd, analog]
(Pilih 6 saja)
- PA1
- PA4 - PA7
- PB0
- PB1

PWM pins [5v, GND, pwm]
- TIM1_CH1 - PA8
- TIM1_CH2 - PA9
- TIM1_CH3 - PA10
- TIM1_CH4 - PA11

! ESP32 (untuk dynamixel) !
- UART0 ke stm32 langsung (pin Rx0 dan Tx0)
- UART2 ke dynamixel rangkaian half duplex (pin Rx2 dan Tx2)
- PIN 4 untuk "enable"

! NANO (untuk MPU) !
- MPU_SDA - pin A4
- MPU_SCL - pin A5
- MPU_INT - pin 2
- UART ke stm32 langsung (pin 0 dan 1)
