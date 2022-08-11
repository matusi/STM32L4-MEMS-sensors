/*
 * ICG20330.c
 *
 *  Created on: Nov 15, 2021
 *      Author: Gaming
 */
#include "ICG20330.h"
uint8_t ICG20330_Initialise( ICG *dev, I2C_HandleTypeDef *i2cHandle, ICG_header *header ) {

	/* init struct parameters */
		dev->i2cHandle 		= i2cHandle;

		dev->i2cHandle 		= i2cHandle;
		memset(dev->IcgGyro, 0,6);
		memset(dev->IcgTemp, 0,2);


		/* Store number of transaction errors (to be returned at end of function) */
		uint8_t errNum = 0;
		HAL_StatusTypeDef status;

	/*
	 */
	uint8_t regData;

	/*
	 * Set output data rate (ODR) and digital filters  BW0 and BW1 Measurement time to 8ms  (DATASHEET PAGE 15)
	 */
	regData = 0x00;

	status = ICG20330_WriteRegister( dev, ICG20330_REG_USER_CTRL, &regData);
	errNum += ( status != HAL_OK );
//	memcpy(header->odr, regData, sizeof(regData)); /// CONVERT TO FLOAT IN MAIN "!"!!!

	/*
	 * Put sensor into ONESHOT mode  *AutoSet disabledd *continuous mode diabled  (DATASHEET PAGE 16 )
	 */
	regData = 0x01;

	status = ICG20330_WriteRegister( dev, ICG20330_REG_PWR_MGMT_1, &regData);
	errNum += ( status != HAL_OK );
	/*
	 * Put sensor into ONESHOT mode  *AutoSet disabledd *continuous mode diabled  (DATASHEET PAGE 16 )
	*/
	regData = 0x00;

	status = ICG20330_WriteRegister( dev, ICG20330_REG_PWR_MGMT_2, &regData);
	errNum += ( status != HAL_OK );
	/*
	 * Put sensor into ONESHOT mode  *AutoSet disabledd *continuous mode diabled  (DATASHEET PAGE 16 )
	 */
	regData = 0x18;

	status = ICG20330_WriteRegister( dev, ICG20330_REG_GYRO_CONFIG, &regData);
	errNum += ( status != HAL_OK );
	/*
	* Put sensor into ONESHOT mode  *AutoSet disabledd *continuous mode diabled  (DATASHEET PAGE 16 )
	*/
	regData = 0x00;

	status = ICG20330_WriteRegister( dev, ICG20330_REG_CONFIG, &regData);
	errNum += ( status != HAL_OK );
	/*
	 * Put sensor into ONESHOT mode  *AutoSet disabledd *continuous mode diabled  (DATASHEET PAGE 16 )
	 */
	regData = 0x00;

	status = ICG20330_WriteRegister( dev, ICG20330_REG_SMPLRT_DIV, &regData);
	errNum += ( status != HAL_OK );

	/* Return number of errors (0 if successful initialisation) */
	return errNum;

}

HAL_StatusTypeDef ICG20330_ReadTemperature( ICG *dev ) {

	/* DATASHEET PAGE 33 */

	/*
	 * Read raw values from temperature registers (16 bits)
	 */
	uint8_t regData[2];

	HAL_StatusTypeDef status = ICG20330_ReadRegisters( dev, ICG20330_REG_TEMP_OUT_H, regData, 2 );


	/*
		 * Combine register values to give raw temperature reading (16 bits)
		 */
//		uint16_t tempRaw = ( (regData[1]  << 8) | regData[0] );

	memcpy(dev->IcgTemp, regData, sizeof(dev->IcgTemp));

	return status;



}
HAL_StatusTypeDef ICG20330_ReadGyro( ICG *dev ) {

	/* DATASHEET PAGE 33 and 34 */

	/*
	 * Read raw values from acceleration registers (x, y, z -> 24 bits each)
	 */
	uint8_t regData[6];

	HAL_StatusTypeDef status = ICG20330_ReadRegisters( dev, ICG20330_REG_GYRO_XOUT_H, regData, 6 );

	/*
		 * Combine register valeus to give raw gyro readings (16 bits each)
		 */
//
//		uint16_t angRaw[3];
//
//		angRaw[0] = (uint16_t) ( (regData[0] << 8) |  (regData[1] ));  /* X-axis */
//		angRaw[1] = (uint16_t) ( (regData[2] << 8) |  (regData[3] ));  /* Y-axis */
//		angRaw[2] = (uint16_t) ( (regData[4] << 8) |  (regData[5] ));  /* Z-axis */

//
//		dev->gyro[0] = angRaw[0];
//		dev->gyro[1] = angRaw[0];
//		dev->gyro[2] = angRaw[0];

	memcpy(dev->IcgGyro, regData, sizeof(dev->IcgGyro));
	return status;

}
uint8_t ICG20330_check_id( ICG *dev, I2C_HandleTypeDef *i2cHandle,ICG_header *header ) {

	/* init struct parameters */
	dev->i2cHandle 	= i2cHandle;

	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef ret;

	/*
	 * Check device, mems, and part IDs (DATASHEET PAGE 26)
	 */
	uint8_t regData;

	ret = ICG20330_ReadRegister( dev, ICG20330_REG_WHO_AM_I, &regData );
	errNum += ( ret != HAL_OK );

	if ( regData != ICG20330_DEVICE_ID ) {

		return 255;

	}
	memcpy(header->chipID, regData, sizeof(header->chipID));
	return errNum;
}
/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ICG20330_ReadRegister( ICG *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, ICG20330_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef ICG20330_ReadRegisters( ICG *dev, uint8_t reg, uint8_t *data, uint8_t length ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, ICG20330_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef ICG20330_WriteRegister( ICG *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Write( dev->i2cHandle, ICG20330_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

