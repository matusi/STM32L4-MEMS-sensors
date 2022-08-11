/*
 * ADXL355.c
 *
 *  Created on: Nov 10, 2021
 *      Author: Gaming
 */

#include "ADXL355.h"
#include <string.h>
uint8_t ADXL355_Initialise( ADXL355 *dev, I2C_HandleTypeDef *i2cHandle , ADXL355_header *header)
{

	/* init struct parameters */

	dev->i2cHandle 		= i2cHandle;
	memset(dev->AdxlAcc, 0, 9);
	memset(dev->AdxlTemp, 0, 2);
	//memset(header->headerStartCode, 0xA5, 1);




	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 * Check device, mems, and part IDs (DATASHEET PAGE 32)
	 */
	uint8_t regData;

	status = ADXL355_ReadRegister( dev, ADXL355_REG_DEVID_AD, &regData );
	errNum += ( status != HAL_OK );

	if ( regData != ADXL355_DEVICE_ID )
	{

		return 255;

	}
	else
	{		memcpy(header->devid_ad, regData, sizeof(regData));

	}

	status = ADXL355_ReadRegister( dev, ADXL355_REG_DEVID_MST, &regData );
	errNum += ( status != HAL_OK );

	if ( regData != ADXL355_MEMS_ID ) {

		return 255;

	}
	else
		{
		memcpy(header->devid_mst, regData, sizeof(regData));

		}

	status = ADXL355_ReadRegister( dev, ADXL355_REG_PARTID, &regData );
	errNum += ( status != HAL_OK );

	if ( regData != ADXL355_PART_ID ) {

		return 255;

	}
	else
		{
		memcpy(header->partid, regData, sizeof(regData));
		}

	/*
		 * Set output data ready to INT1)
		 */
		regData = 0x09;

		status = ADXL355_WriteRegister( dev, ADXL355_REG_INT_MAP, &regData);
		errNum += ( status != HAL_OK );
	/*
	 *  INT PIN ACTIVE HIGH IN RANGE REG  data sheet page 39
	 */
		regData = 0xc1;
		status = ADXL355_WriteRegister( dev, ADXL355_REG_RANGE, &regData);
		errNum += ( status != HAL_OK );


		/*
			 * Set output data rate (ODR) and digital filters (no high-pass filter, 500 Hz ODR, 125 Hz low-pass filter cut-off) (DATASHEET PAGE 37)
			 */


		switch ((0x03 & regData))
		   {
				case 1: header->range = 2;
						break;
				case 2: header->range = 4;
						break;
				case 3: header->range = 8;
						break;
				default: header->range = 0;
			}


	regData = 0x03;

	status = ADXL355_WriteRegister( dev, ADXL355_REG_FILTER, &regData);
	errNum += ( status != HAL_OK );
//	float odr =4000.0 / pow(2, (regData & 0x0F)) ;
//	float lpf = 1000.0 / pow(2, (regData & 0x0F));
//	memcpy(header->odr,odr, sizeof(odr));
//	memcpy(header->lpf,lpf, sizeof(lpf));


	/*
	 * Put sensor into measurement mode (DATASHEET PAGE 38)
	 */
	regData = 0x00;

	status = ADXL355_WriteRegister( dev, ADXL355_REG_POWER_CTL, &regData);
	errNum += ( status != HAL_OK );


	/* Return number of errors (0 if successful initialisation) */
	if ( errNum != 0 )
	{return errNum;}
	else {
		return header;
	}


}
/*
 * DATA ACQUISITION
 */

HAL_StatusTypeDef ADXL355_ReadTemperature( ADXL355 *dev ) {

	/* DATASHEET PAGE 33 */

	/*
	 * Read raw values from temperature registers (16 bits)
	 */
	uint8_t regData[2];
	HAL_StatusTypeDef status = ADXL355_ReadRegisters( dev, ADXL355_REG_TEMP2, regData, 2 );


	memcpy(dev->AdxlTemp, regData, sizeof(regData));

	return status;

}

HAL_StatusTypeDef ADXL355_ReadAccelerations( ADXL355 *dev ) {

	/* DATASHEET PAGE 33 and 34 */

	/*
	 * Read raw values from acceleration registers (x, y, z -> 24 bits each) from Reg XDATA3 &ADD 0x08 to reg ZDATA3 &ADD 0x10
	 */
	uint8_t regData[9];

	HAL_StatusTypeDef status = ADXL355_ReadRegisters( dev, ADXL355_REG_XDATA3, regData, 9 );

	memcpy(dev->AdxlAcc, regData, sizeof(regData));

	return status;

}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADXL355_ReadRegister( ADXL355 *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, ADXL355_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef ADXL355_ReadRegisters( ADXL355 *dev, uint8_t reg, uint8_t *data, uint8_t length ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, ADXL355_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef ADXL355_WriteRegister( ADXL355 *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Write( dev->i2cHandle, ADXL355_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}
