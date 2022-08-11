/*
 * BMG250.c
 *
 *  Created on: Dec 16, 2021
 *      Author: Gaming
 */


/*
 * ICG20330.c
 *
 *  Created on: Nov 15, 2021
 *      Author: Gaming
 */
#include "BMG250.h"
//HAL_StatusTypeDef BMG250_device_reset( BMG *dev, I2C_HandleTypeDef *i2cHandle)
//{
//
//
//	/* init struct parameters */
//	dev->i2cHandle 		= i2cHandle;
//
//	/* Store number of transaction errors (to be returned at end of function) */
//
//		HAL_StatusTypeDef ret;
//		uint8_t regData;
//
//		regData= HARD_RESET_CMD; /* initialise the regData with The hard reset cmd  0x80*/
//		ret =BMG250_WriteRegister( dev, MMC5983MA_REG_ICR1, &regData);
//
//		if(ret != HAL_OK)
//		{
//			return ret;
//		}
//
//		HAL_Delay(200);
//		return HAL_OK;
//	}

uint8_t BMG250CheckId( BMG *dev, I2C_HandleTypeDef *i2cHandle, BMG250_header *header) {

	/* init struct parameters */
	dev->i2cHandle 	= i2cHandle;

	/*header struct  initalization  */
memset(header->headerStartCode, BMG250_HEADER_START_CODE, sizeof(header->headerStartCode));
	//memset(header->headerStartCode, 0xB0, 1);


	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef ret;

	/*
	 * Check device, mems, and part IDs (DATASHEET PAGE 26)
	 */
	uint8_t regData;

	ret = BMG250_ReadRegister( dev, BMG250_REG_CHIP_ID, &regData );
	errNum += ( ret != HAL_OK );

	if ( regData != BMG250_DEVICE_ID ) {

		return 255;
	}
	if ( errNum != 0 )
		{return errNum;}
		else {
			memcpy(header->chipID, regData, sizeof(regData));
}

uint8_t BMG250_read_pmu_status( BMG *dev, I2C_HandleTypeDef *i2cHandle ) {

	/* init struct parameters */
	dev->i2cHandle 	= i2cHandle;

	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef ret;

	/*
	 * Check device, mems, and part IDs (DATASHEET PAGE 26)
	 */
	uint8_t regData;

	ret = BMG250_ReadRegister( dev, BMG250_REG_PMU_STATUS, &regData );
	errNum += ( ret != HAL_OK );

	}
	return errNum;
}
uint8_t BMG250_read_err_reg( BMG *dev, I2C_HandleTypeDef *i2cHandle ) {

	/* init struct parameters */
	dev->i2cHandle 	= i2cHandle;

	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef ret;

	/*
	 * Check device, mems, and part IDs (DATASHEET PAGE 26)
	 */
	uint8_t regData;

	ret = BMG250_ReadRegister( dev, BMG250_REG__ERR_REG, &regData );
	errNum += ( ret != HAL_OK );


	return errNum;
}
uint8_t BMG250_self_test( BMG *dev, I2C_HandleTypeDef *i2cHandle ) {




	/* init struct parameters */
		dev->i2cHandle 		= i2cHandle;
//		memset(dev->BmgGyro, 0, 6);
//		memset(dev->BmgTemp, 0, 2);
//		memset(header->headerStartCode, 0xA5, 1);



	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 */
	uint8_t regData;
	uint8_t regDatain;

	/*
		 * enable drdy int bit
		 */
		regData = 0x10;

		status = BMG250_WriteRegister( dev, BMG250_REG_SELF_TEST, &regData);
		errNum += ( status != HAL_OK );
	HAL_Delay(50);


	status = BMG250_ReadRegister( dev, BMG250_REG_STATUS, &regDatain );
		errNum += ( status != HAL_OK );

		if ( regData != BMG250_DEVICE_ID ) {

			return 255;

		}
		/*	*/
		return errNum;




}


//uint8_t BMG250_power_mode( BMG *dev, I2C_HandleTypeDef *i2cHandle ) {
//
//	/* init struct parameters */
//	dev->i2cHandle 		= i2cHandle;
//
//	/* Store number of transaction errors (to be returned at end of function) */
//	uint8_t errNum = 0;
//	HAL_StatusTypeDef status;
//
//	/*
//	 */
//	uint8_t regData;
//	/*
//SET POWER MODE REG 		 */
//		regData = 0x11;
//
//		status = BMG250_WriteRegister( dev, 0x7E, &regData);
//		errNum += ( status != HAL_OK );
//
//		/*	*/
//		return errNum;
//
//
//
//
//}

uint8_t BMG250_SoftResetAndInit( BMG *dev, I2C_HandleTypeDef *i2cHandle ,BMG250_header *header){



/* init struct parameters */
dev->i2cHandle 		= i2cHandle;



		/* Data sturct initilisation  */
	memset(dev->dataStartCode, BMG250_DATA_START_CODE,sizeof(dev->dataStartCode));
	memset(dev->BmgGyro, 0,6);
	memset(dev->BmgTemp, 0,2);
	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 */
	uint8_t regData;
		/*
		 * soft reset
		 */
	regData = 0xB6;

		status = BMG250_WriteRegister( dev, 0x7E, &regData);
		errNum += ( status != HAL_OK );

		/*	*/
		HAL_Delay(100);
		/*
		 * set gyro normal mode
		 */

	regData = 0x15;

		status = BMG250_WriteRegister( dev, 0x7E, &regData);
		errNum += ( status != HAL_OK );

				/*	*/
		HAL_Delay(100);

		/*
		 * FIFO CONFIG
		 */

	regData = 0x80;

		status = BMG250_WriteRegister( dev, BMG250_REG_FIFO_CONFIG_1, &regData);
		errNum += ( status != HAL_OK );
		/*
		 * Enable drdy
		 */

	regData = 0x10;

		status = BMG250_WriteRegister( dev, BMG250_REG_INT_EN_1, &regData);
		errNum += ( status != HAL_OK );
		/*
		 * config INT2 Pin pushpull..
		 */

	regData = 0xB0;

		status = BMG250_WriteRegister( dev, BMG250_REG_INT_OUT_CTRL, &regData);
		errNum += ( status != HAL_OK );
		/*
		 * map drdy to int2
		*/

	regData = 0x08;

		status = BMG250_WriteRegister( dev, BMG250_REG_INT_MAP_1, &regData);
		errNum += ( status != HAL_OK );
		/*
		 * map drdy to int2
		*/

	regData = 0x2A;

		status = BMG250_WriteRegister( dev, BMG250_REG_GYR_CONF, &regData);
		errNum += ( status != HAL_OK );
		/*
		 * map drdy to int2
		*/

//	regData = 0x08;
//
//		status = BMG250_WriteRegister( dev, BMG250_REG_INT_MAP_1, &regData);
//		errNum += ( status != HAL_OK );
//		/*
//		 * map drdy to int2
//		*/

	regData = 0x05;

		status = BMG250_WriteRegister( dev, BMG250_REG_FIFO_DOWNS, &regData);
		errNum += ( status != HAL_OK );
		/*
		 * map drdy to int2
		*/
//
//	regData = 0x01;
//
//		status = BMG250_WriteRegister( dev, BMG250_REG_GYR_RANGE, &regData);
//		errNum += ( status != HAL_OK );
//							/*	*/
//		return errNum;

		/*
		 	 * Set output data rate (ODR) and digital filters ODR= 100HZ  LPF=39,9 NORMAL MODE  (DATASHEET PAGE 15)
		 */
				regData = 0x28;

		status = BMG250_WriteRegister( dev, BMG250_REG_CONF, &regData);
				errNum += ( status != HAL_OK );
				float odr = regData ;// calculate Odr  float
				/* Update ODR to Header */
			//	memcpy(header->odr, odr, sizeof(odr));



		/*
		 	 * set full scale range 2000 deg /s  lsb 16,4 lsb/deg/s
		*/
				regData = 0x00;

		status = BMG250_WriteRegister( dev, BMG250_REG_GYR_RANGE, &regData);
				errNum += ( status != HAL_OK );

				float range = regData  ;// calcute odr ADD formula and MASK
				/* Update ODR to Header */
				//memcpy(header->range, range, sizeof(range)); FIX THIS !!! MEM COPY WITH FLOAT !!!!!  USED INT THEN CALCULTE IN MAIN JUST FIX THIS


			/*
			SET POWER MODE REG 		 */
					regData = 0x11;

					status = BMG250_WriteRegister( dev, 0x7E, &regData);
					errNum += ( status != HAL_OK );

					/*	*/
					return errNum;


}
//uint8_t BMG250_Initialise( BMG *dev, I2C_HandleTypeDef *i2cHandle,BMG250_header *header  ){
//
//	/* init struct parameters */
//	dev->i2cHandle 		= i2cHandle;
//
//
//	memset(dev->BmgGyro, 0, 6);
//	memset(dev->BmgTemp, 0, 2);
//
//
//	/* Store number of transaction errors (to be returned at end of function) */
//	uint8_t errNum = 0;
//	HAL_StatusTypeDef status;
//
//	/*
//	 */
//	uint8_t regData;
//	/*
//	  	  * ENABLE The Data ready DRDY INT BIT
//	*/
//			regData = 0x10;
//
//	status = BMG250_WriteRegister( dev, BMG250_REG_INT_EN_1, &regData);
//			errNum += ( status != HAL_OK );
//	/*
//		  * ENABLE the sensor's interruption pin INT1
//	*/
//			regData = 0x08;
//
//	status = BMG250_WriteRegister( dev, BMG250_REG_INT_OUT_CTRL, &regData);
//			errNum += ( status != HAL_OK );
//	/*
//	 	  * MAP The DRDY INT to INT1 PIN
//	*/
//			regData = 0x80;
//
//	status = BMG250_WriteRegister( dev, BMG250_REG_INT_MAP_1, &regData);
//			errNum += ( status != HAL_OK );
//
//	/*
//	 	 * Set output data rate (ODR) and digital filters ODR= 100HZ  LPF=39,9 NORMAL MODE  (DATASHEET PAGE 15)
//	 */
//			regData = 0x28;
//
//	status = BMG250_WriteRegister( dev, BMG250_REG_CONF, &regData);
//			errNum += ( status != HAL_OK );
//			float odr = regData ;// calculate Odr  float
//			/* Update ODR to Header */
//			memcpy(header->odr, odr, sizeof(odr));
//
//
//
//	/*
//	 	 * set full scale range 2000 deg /s  lsb 16,4 lsb/deg/s
//	*/
//			regData = 0x00;
//
//	status = BMG250_WriteRegister( dev, BMG250_REG_GYR_RANGE, &regData);
//			errNum += ( status != HAL_OK );
//
//			float range = regData  ;// calcute odr ADD formula and MASK
//			/* Update ODR to Header */
//			memcpy(header->range, range, sizeof(range));
//
//	return errNum;
//
//}

HAL_StatusTypeDef BMG250_ReadTemperature( BMG *dev ) {

	/* DATASHEET PAGE 33 */

	/*
	 * Read raw values from temperature registers (16 bits)
	 */
	uint8_t regData[2];

	HAL_StatusTypeDef status = BMG250_ReadRegisters( dev, BMG250_REG_TEMPERATURE_0, regData, 2 );


	/*
		 * Combine register values to give raw temperature reading (16 bits)
		 */
//		uint16_t tempRaw = ( (regData[1]  << 8) | regData[0] );
	memcpy(dev->BmgTemp, regData, sizeof(regData));
	return status;

}

HAL_StatusTypeDef BMG250_ReadGyro( BMG *dev ) {

	/* DATASHEET PAGE 33 and 34 */

	/*
	 * Read raw values from acceleration registers (x, y, z -> 24 bits each)*/
	uint8_t regData[6];

	HAL_StatusTypeDef status = BMG250_ReadRegisters( dev, BMG250_REG_GYR_X_0_7, regData, 6 );

	memcpy(dev->BmgGyro, regData, sizeof(regData));


	return status;


}
HAL_StatusTypeDef BMG250_ReadGyro_DMA(BMG *dev) {

HAL_StatusTypeDef status = BMG250_ReadRegisters_DMA( dev, BMG250_REG_GYR_X_0_7, dev->BmgGyro, 6 );

//dev->readingGyr = 1;
			return status;


}


HAL_StatusTypeDef BMG250_ReadTemp_DMA(BMG *dev) {

HAL_StatusTypeDef status = BMG250_ReadRegisters_DMA( dev, BMG250_REG_TEMPERATURE_0,dev->BmgTemp, 2 );

			return status;


}
void BMG250_ReadGyroscopeDMA_Complete(BMG *dev) {

//	HAL_GPIO_WritePin(imu->csGyrPinBank, imu->csGyrPin, GPIO_PIN_SET);
	//dev->readingGyr = 0;

//	/* Form signed 16-bit integers */
//	int16_t gyrX = (int16_t) ((imu->gyrRxBuf[2] << 8) | imu->gyrRxBuf[1]);
//	int16_t gyrY = (int16_t) ((imu->gyrRxBuf[4] << 8) | imu->gyrRxBuf[3]);
//	int16_t gyrZ = (int16_t) ((imu->gyrRxBuf[6] << 8) | imu->gyrRxBuf[5]);
//
//	/* Convert to deg/s */
//	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
//	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
//	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;
//	/*
//		 * Combine register valeus to give raw gyro readings (16 bits each)
//		 */
//
//		uint16_t angRaw[3];
//
//		angRaw[0] = (uint16_t) ( (regData[1] << 8) |  (regData[0] ));  /* X-axis */
//		angRaw[1] = (uint16_t) ( (regData[3] << 8) |  (regData[2] ));  /* Y-axis */
//		angRaw[2] = (uint16_t) ( (regData[5] << 8) |  (regData[4] ));  /* Z-axis */
//
//		dev->BmgGyro[0] = angRaw[0];
//		dev->BmgGyro[1] = angRaw[0];
//		dev->BmgGyro[2] = angRaw[0];
}
/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef BMG250_ReadRegister( BMG *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, BMG250_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef BMG250_ReadRegisters( BMG *dev, uint8_t reg, uint8_t *data, uint8_t length ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, BMG250_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef BMG250_WriteRegister( BMG *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Write( dev->i2cHandle, BMG250_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef BMG250_ReadRegisters_DMA( BMG *dev, uint8_t reg, uint8_t *data, uint8_t length ) {

	return HAL_I2C_Mem_Read_DMA( dev->i2cHandle, BMG250_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length);

}
