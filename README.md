# **STM32L4 MEMS sensors**
 STM32L4  microcontroller  firmware For MEMS  IMU sensors


## API description

This package contains MEMS sensor driver (sensor API) 






Driver files information
bmg160.h
This header file has the register address definition, constant definitions, data type definition and supported sensor driver calls declarations.
bmg160.c
This file contains the implementation for the sensor driver APIs.
bmg160_support.c
This file shall be used as an user guidance, here you can find samples of * Initialize the sensor with I2C/SPI communication - Add your code to the SPI and/or I2C bus read and bus write functions. - Return value can be chosen by yourself - API just passes that value to your application code - Add your code to the delay function - Change I2C address accordingly in bmg160.h
Power mode configuration of the sensor
Get and set functions usage
Reading the sensor read out data

## Sensors characteristics 
- #### ADXL355 Accelerometer 

- #### BMG250 Gyroscope 


- #### ICG20330 Gyroscope 


- #### MMCMA Magnetometer

- #### 89BSD Barometer



## Documentations 


- #### ADXL355 Accelerometer 

- #### BMG250 Gyroscope 


- #### ICG20330 Gyroscope 


- #### MMCMA Magnetometer

- #### 89BSD Barometer
