# **STM32L4 MEMS sensors**
 STM32L4  microcontroller  firmware For MEMS  IMU sensors


## API description

This package contains MEMS sensor driver (sensor API) 

## Driver files information
### Header files

The headers file has the registers address definitions, constants definitions, data types definitions and supported sensor driver calls declarations.
### Source files 
The Source files contains the implementation for the sensors driver APIs.
1. Initialize the sensor with I2C/SPI communication - Add your code to the SPI and/or I2C bus read and bus write functions. - Return value can be chosen by yourself - API just passes that value to your application code - Add your code to the delay function - Change I2C address accordingly.
2. Power mode configuration of the sensor
3. Get and set functions usage
4. Reading the sensor read out data

## Sensors characteristics 


- #### ADXL355 Accelerometer 

- 2-3.6V Supply
- ±250/500/1000/2000°/s configurable range
- Output Data Rates (ODR) from 12.5 to 800 Hz
- 16-bit digital output resolution
- 192 bytes FIFO buffer (32 X/Y/Z samples)


The ADXL355 [4] from Analog Circuits is a low power and low noise accelerometer it
can be set to measure in the ranges ±2g, ±4g, ±8g and the ADC belonging to the sensor is
20 − bit. If the IMU is set to measure up to 8g the accuracy wouldnt be high but it would
be able to measure bigger forces. But if its set to 2g the accuracy would be high but the
20
forces measured is low. To use the accelerometer effectively you must know how high
forces your object is exposed to.

-#### Supported sensor interface
- ADXL355 driver supports SPI and I2C interfaces


- #### BMG250 Gyroscope 

- 2-3.6V Supply
- ±250/500/1000/2000°/s configurable range
- Output Data Rates (ODR) from 12.5 to 800 Hz
- 16-bit digital output resolution
- 192 bytes FIFO buffer (32 X/Y/Z samples)

. The BMG250[3] gyroscope from InvenSense can measure with the
sensitivity of 16.4 LSB ◦/s to 262.4 LSB ◦/s the resolution of the ADC is 16-bit and
the range is from 125 ◦/s to 2000 ◦/s.
-#### Supported sensor interface
- BMG250 driver supports SPI and I2C interfaces



- #### ICG20330 Gyroscope 

- 2-3.6V Supply
- ±250/500/1000/2000°/s configurable range
- Output Data Rates (ODR) from 12.5 to 800 Hz
- 16-bit digital output resolution
- 192 bytes FIFO buffer (32 X/Y/Z samples)


The ICG-20330[2] gyroscope from InvenSense can measure with the sensitivity of
131 LSB ◦/s to 1048 LSB ◦/s the resolution of the ADC is 16-bit and multiple ranges
of range is from 31.5 ◦/s, 125 ◦/s to 250 ◦/s.

-#### Supported sensor interface
- ICG-20330 driver supports SPI and I2C interfaces




- #### MMCMA Magnetometer

- 2-3.6V Supply
- ±250/500/1000/2000°/s configurable range
- Output Data Rates (ODR) from 12.5 to 800 Hz
- 16-bit digital output resolution
- 192 bytes FIFO buffer (32 X/Y/Z samples)


The MMC5983MA[5] can measure magnetic field strength with configurable sensitivity of
0.25 mG up to 0.0625 mG , has an ADC with a resolution of 16-bit up to 18-bit resolution
and a full scale range of ±8 G.

-#### Supported sensor interface
- MMC5983MA driver supports SPI and I2C interfaces


- #### 89BSD Barometer
-   2-3.6V Supply
- ±250/500/1000/2000°/s configurable range
- Output Data Rates (ODR) from 12.5 to 800 Hz
- 16-bit digital output resolution
- 192 bytes FIFO buffer (32 X/Y/Z samples)
-#### Supported sensor interface
- 89BSD driver supports SPI and I2C interfaces
The TE89BSD[6] can measure pressure with a multiple configurable range of 300 mbar
up to 1200 mba ,has an ADC with a resolution of 24-bit and a sensitivity per LSB of 0.016
mbar up to 0.11 mbar.

## Documentations 


- #### ADXL355 Accelerometer 

- #### BMG250 Gyroscope 


- #### ICG20330 Gyroscope 


- #### MMCMA Magnetometer

- #### 89BSD Barometer
