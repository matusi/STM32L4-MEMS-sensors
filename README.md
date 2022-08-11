# **STM32L4 MEMS sensors**
 STM32L4  microcontroller  firmware For MEMS  IMU sensors


## API description

This package contains MEMS sensor driver (sensor API) 

## Driver files information
### Header files

The headers file has the registers address definitions, constants definitions, data types definitions and supported sensor driver calls declarations.
### Source files 
The Source files contains the implementation for the sensors driver APIs.
1. Initialize the sensor with I2C/SPI communication - Add your code to the SPI and/or I2C bus read and bus write functions. - Return value can be chosen by yourself - API just passes that value to your application code - Add your code to the delay function - Change I2C address accordingly in bmg160.h
2. Power mode configuration of the sensor
3. Get and set functions usage
4. Reading the sensor read out data

## Sensors characteristics 
- #### ADXL355 Accelerometer 

- #### BMG250 Gyroscope 

- 2-3.6V Supply
- ±250/500/1000/2000°/s configurable range
- Output Data Rates (ODR) from 12.5 to 800 Hz
- 16-bit digital output resolution
- 192 bytes FIFO buffer (32 X/Y/Z samples)


- #### ICG20330 Gyroscope 


- #### MMCMA Magnetometer

- #### 89BSD Barometer



## Documentations 


- #### ADXL355 Accelerometer 

- #### BMG250 Gyroscope 


- #### ICG20330 Gyroscope 


- #### MMCMA Magnetometer

- #### 89BSD Barometer
