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

- 2.25 V to 3.6 V Supply
- Integrated temperature sensor
- ±2, ±4, ±8 g configurable range
- Output Data Rates (ODR) from 3.906 to 4000 Hz 
- 20-bit digital output resolution
-   96 locations of 20 bit FIFO buffer (X/Y/Z samples)
- Programmable high- and low-pass digital filters
-#### Supported sensor interface
- ADXL355 driver supports SPI and I2C interfaces


- #### BMG250 Gyroscope 

- : 1.71V to 3.6VSupply
- ±125/250/500/1000/2000°/s configurable range
- 131 LSB ◦/s to 1048 LSB ◦/s sensitivity 
- Output Data Rates (ODR) from 25 to 3200 Hz
- 16-bit digital output resolution
- Allocatable FIFO buffer of 1024 bytes
- BMG250 driver supports SPI and I2C interfaces

- #### ICG20330 Gyroscope 

- 1.71 V to 3.6 V Supply
- ±31.25/ ±62.5/ ±125 and ±250°/s configurable range
- Output Data Rates (ODR) from 1000 to 8000 Hz
- 131 LSB ◦/s to 1048 LSB ◦/s sensitivity 
- 16-bit digital output resolution
- 512 bytes FIFO buffer 
- ICG-20330 driver supports SPI and I2C interfaces




- #### MMC5983MA Magnetometer

- 2-3.6V Supply
-±8/ G. configurable full scale range 
- Output Data Rates (ODR) from 12.5 to 800 Hz
- 16-bit digital output resolution
- 192 bytes FIFO buffer (32 X/Y/Z samples)

- 1.71 V to 3.6 V Supply
- ±31.25/ ±62.5/ ±125 and ±250°/s configurable range
- Output Data Rates (ODR) from 1000 to 8000 Hz
- 0.25 mG up to 0.0625 mG  configurable  sensitivity 
- 16-bit up to 18-bit digital output resolution

- 512 bytes FIFO buffer 
- MMC5983MA driver supports SPI and I2C interface


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
[ ADXL355 product overview](https://www.analog.com/en/products/adxl355.html#product-overview) (datasheet, evaluation board schematic)

- #### BMG250 Gyroscope 


- #### ICG20330 Gyroscope 


- #### MMCMA Magnetometer

- #### 89BSD Barometer
