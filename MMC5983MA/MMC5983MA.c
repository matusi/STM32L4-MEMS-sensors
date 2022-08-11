/*
 * MMC5983MA.c
 *
 *  Created on: Nov 10, 2021
 *      Author: Gaming
 */

#include "MMC5983MA.h"

/*
 * ms58driver.c
 *
 *  Created on: Sep 16, 2021
 *      Author: Gaming
 */

#include "math.h"
#include "stm32l4xx_hal.h"
#include <assert.h>
#include <stdio.h>


HAL_StatusTypeDef MmcHardReset( MMC5983 *dev, I2C_HandleTypeDef *i2cHandle)
{


	/* init struct parameters */
	dev->i2cHandle 		= i2cHandle;

	/* Store number of transaction errors (to be returned at end of function) */

		HAL_StatusTypeDef ret;
		uint8_t regData;

		regData= HARD_RESET_CMD; /* initialise the regData with The hard reset cmd  0x80*/
		ret =MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR1, &regData);

		if(ret != HAL_OK)
		{
			return ret;
		}

		HAL_Delay(200);
		return HAL_OK;
	}


uint8_t MmcCheckID( MMC5983 *dev, I2C_HandleTypeDef *i2cHandle, MMC_header *header ) {

	/* init struct parameters */
	dev->i2cHandle 	= i2cHandle;

	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef ret;

	/*
	 * Check device, mems, and part IDs (DATASHEET PAGE 32)
	 */
	uint8_t regData;

	ret = MMC5983MA_ReadRegister( dev, MMC5983MA_REG_ID, &regData );
	errNum += ( ret != HAL_OK );

	if ( regData != MMC5983MA_DEVICE_ID ) {

		return 255;

	}
	memcpy(header->chipID, regData, sizeof(header->chipID));
	return errNum;
}




uint8_t MmcInitialiseOneShot( MMC5983 *dev, I2C_HandleTypeDef *i2cHandle,MMC_header *header  ) {

	/* init struct parameters */
	dev->i2cHandle 		= i2cHandle;


	memset(dev->MmcMag , 0, 7);
	memset(dev->MmcTemp, 0, 1);


	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 */
	uint8_t regData;

	/*
	 * Set output data rate (ODR) Measuremment Time = 8ms  and digital filters Bandwidth 100HZ  BW0 and BW1 Measurement time to 8ms  (DATASHEET PAGE 15)
	 */
	regData = MBW_100Hz;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR1, &regData);
	errNum += ( status != HAL_OK );
	//memcpy(header->range, regData, sizeof(header->range));//   FIX THIS  !!! TO UPDATE RANGE AS FLOAT OR HEX VALUE NEEDED ??
	/*
	 * Put sensor into ONESHOT mode  *AutoSet disabledd *continuous mode diabled  (DATASHEET PAGE 16 )
	 */
	regData = MODR_ONESHOT;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR2, &regData);
	errNum += ( status != HAL_OK );

	/* Return number of errors (0 if successful initialisation) */
	return errNum;

}
uint8_t MmcConfigSet( MMC5983 *dev ) {


	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 */
	uint8_t regData;

	/*
	 * Enable Set operation bit in CR3 (DATASHEET PAGE 16)
	 */
	regData = SET_ENABLE;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR3, &regData);
	errNum += ( status != HAL_OK );

	/*
	 * Set bit in CR0  (DATASHEET PAGE 15 )
	 */
	regData = SET_START;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR0, &regData);
	errNum += ( status != HAL_OK );
	/* wait 200 ms */
	HAL_Delay(200);




	/* Return number of errors (0 if successful initialisation) */
	return errNum;

}

HAL_StatusTypeDef MmcGetSetField( MMC5983 *dev , MMC_header *header)
{

	/* Store number of transaction errors (to be returned at end of function) */
		uint8_t errNum = 0;
		HAL_StatusTypeDef status;

	/*
	 * Read raw values from acceleration registers (x, y, z -> 24 bits each)
	 */
	uint8_t regData_send ;
	uint8_t regData[7];

	uint8_t regDatacheck[1]={0};

	/*
		 * Set Take Measearement _Mag to 1 CR0  (DATASHEET PAGE 15 )
	 */
	regData_send = MAG_MEASUR_START;

		status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR0, &regData_send);
		errNum += ( status != HAL_OK );


		/*
		 * Read raw values from x,y,z registers (x, y, z -> 18 bits each)
		 */



		/*
			 * Read status register
			*/

			while (regDatacheck[0] != DATA_MAG_READY)
			{
				 status = MMC5983MA_ReadRegister( dev, MMC5983MA_REG_STATUS, regDatacheck );
			}
			errNum += ( status != HAL_OK );



	 status = MMC5983MA_ReadRegisters( dev, MMC5983MA_REG_X0OUT, regData, 7 );
	 errNum += ( status != HAL_OK );
	 if (status ==HAL_OK){ memcpy(header->SetField, regData,sizeof(header->SetField));}
}
uint8_t MmcConfigReset( MMC5983 *dev ) {


	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/*
	 */
	uint8_t regData;

	/*
	 * Enable Reset operation bit in CR3 (DATASHEET PAGE 16)
	 */
	regData = RESET_ENABLE;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR3, &regData);
	errNum += ( status != HAL_OK );

	/*
	 * Reset bit in CR0  (DATASHEET PAGE 15 )
	 */
	regData = RESET_START;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR0, &regData);
	errNum += ( status != HAL_OK );
	/* wait 200 ms */
	HAL_Delay(200);
	/* Return number of errors (0 if successful initialisation) */
	return errNum;

}


HAL_StatusTypeDef MmcGetResetField( MMC5983 *dev , MMC_header *header)
{

	/* Store number of transaction errors (to be returned at end of function) */
		uint8_t errNum = 0;
		HAL_StatusTypeDef status;

	/*
	 * Read raw values from acceleration registers (x, y, z -> 24 bits each)
	 */
	uint8_t regData_send ;
	uint8_t regData[7];

	uint8_t regDatacheck[1]={0};

	/*
		 * Set Take Measearement _Mag to 1 CR0  (DATASHEET PAGE 15 )
	 */
	regData_send = MAG_MEASUR_START;

		status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR0, &regData_send);
		errNum += ( status != HAL_OK );


		/*
		 * Read raw values from x,y,z registers (x, y, z -> 18 bits each)
		 */



		/*
			 * Read status register
			*/

			while (regDatacheck[0] != DATA_MAG_READY)
			{
				 status = MMC5983MA_ReadRegister( dev, MMC5983MA_REG_STATUS, regDatacheck );
			}
			errNum += ( status != HAL_OK );



	 status = MMC5983MA_ReadRegisters( dev, MMC5983MA_REG_X0OUT, regData, 7 );
	 errNum += ( status != HAL_OK );
	 if (status ==HAL_OK){
	 memcpy(header->ResetField, regData,sizeof(header->ResetField));}
}
uint8_t MmcConfigContiniousINT( MMC5983 *dev ,MMC_header *header) {


	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t regData = 0;


	/**Enable INT Measurement Done BIT IN CR0 */
	regData = 0x04;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR0, &regData);
	errNum += ( status != HAL_OK );

	/*
	 * Set digital filters Bandwidh  BW0 and BW1  to 100HZ Measurement Duration to 8ms  (DATASHEET PAGE 15)
	 * Range is a function of the ( Resolution 18 bit and the Measurement duration
	 * here we return the Hex value of the  Selected measurement Time
	 *Resolution is 18 Bit
	 */
	regData = MBW_100Hz;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR1, &regData);
	errNum += ( status != HAL_OK );
//	 if (status ==HAL_OK){
	// memcpy(header->range, regData,sizeof(header->range));} FIX THIS  HEX OR FLOAT )??? ALSO FOR SET AND RESET FIELD ??
	/*
	 * Set output data rate (ODR) TO 100HZ  (DATASHEET PAGE 15)
	 */
	regData = MODR_100Hz;

	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR2, &regData);
	errNum += ( status != HAL_OK );
//	 if (status ==HAL_OK){
	// memcpy(header->odr, regData,sizeof(header->odr));}

	/* Return number of errors (0 if successful initialisation) */
	return errNum;

}




HAL_StatusTypeDef MmcReadMagField( MMC5983 *dev ) {

	/* Store number of transaction errors (to be returned at end of function) */
		uint8_t errNum = 0;
		HAL_StatusTypeDef status;

	/*
	 * Read  HEX raw values from magnetic field  registers (x, y, z -> 18 bits each)
	 */
	uint8_t regData[7];
	 status = MMC5983MA_ReadRegisters( dev, MMC5983MA_REG_X0OUT, regData, 7 );
	 errNum += ( status != HAL_OK );
 if (status ==HAL_OK){
		 memcpy(dev->MmcMag,regData,sizeof(dev->MmcMag));}




	return status;

}

HAL_StatusTypeDef MmcReadTemperature( MMC5983 *dev ) {

	/* Store number of transaction errors (to be returned at end of function) */
		uint8_t errNum = 0;
		HAL_StatusTypeDef status;

	/*
	 * Read  HEX raw values from magnetic field  registers (x, y, z -> 18 bits each)
	 */
	uint8_t regData ;
	 status = MMC5983MA_ReadRegisters( dev, MMC5983MA_REG_TOUT, regData, 7 );
	 errNum += ( status != HAL_OK );
	 if (status ==HAL_OK){
		 memcpy(dev->MmcTemp,regData,sizeof(dev->MmcMag));
	 }

	return status;

}

//HAL_StatusTypeDef check_for_mag_measurement(uint8_t MMC5983MA_ADDRESS, uint8_t MMC5983MA_STATUS,uint8_t  data_ready){
//
//
//HAL_StatusTypeDef ret;
//uint8_t rawdata[1]= {0};
//// Get c1 data, reg A2, two bytes
//
//if (rawdata != data_ready)
//{
//ret = HAL_I2C_Mem_Read(&hi2c1, MMC5983MA_ADDRESS |0x01, MMC5983MA_STATUS, 1,  rawdata, 1, 50);
//if(ret != HAL_OK)
//{
//	return ret;
//}
//}
//return HAL_OK;
//
//}





//
//HAL_StatusTypeDef  calculalte_ref(uint8_t MMC5983MA_ADDRESS,uint8_t MMC5983MA_ZOUT_0,uint8_t  MMC5983MA_XYZOUT_2 ,uint32_t *z_reset)
//{
//
//
//
//	    if (x_set > x_reset)
//	    {
//	    	ref_x = x_set - x_reset;
//	    }
//	    else
//	    {
//	    	ref_x = x_reset -  x_set;
//	    }
//
//	    if (y_set > y_reset)
//	    	    {
//	    	    	ref_y = y_set - y_reset;
//	    	    }
//	    	    else
//	    	    {
//	    	    	ref_y = y_reset -  y_set;
//	    	    }
//	    if (z_set > z_reset)
//	    	    {
//	    	    	ref_z = z_set - z_reset;
//	    	    }
//	    	    else
//	    	    {
//	    	    	ref_z= z_reset -  z_set;
//	    	    }
//
//
//}


//HAL_StatusTypeDef  calculalte_offset(float *off)
//{
//
//	for (uint8_t i = 0; i < 3; i++)
//	   {
//	      off[i] = ((xset[i] + xreset[i])/2);
//	   }
//
//}
//HAL_StatusTypeDef  Perform_mag_measurement(float *xset,float *yset,float *zset)
//{
//	HAL_StatusTypeDef ret;
//	uint8_t rawdata[7]= {0};
//	// Get c1 data, reg A2, two bytes
//
//
//
//	uint8_t ready_data[1]={0x00};
//	// Get c1 data, reg A2, two bytes
//
//	if (ready_data[0] != DATA_MAG_READY)
//	{
//	ret = HAL_I2C_Mem_Read(&hi2c1, MMC5983MA_ADDRESS |0x01, MMC5983MA_STATUS, 1, ready_data, 1, 50);
//	*ready_data = ready_data;
//	if(ret != HAL_OK)
//	{
//		return ret;
//	}
//	}
//	ret = HAL_I2C_Mem_Read(&hi2c1, MMC5983MA_ADDRESS |0x01, MMC5983MA_XOUT_0, 1,  rawdata, 7, 50);
//	if(ret != HAL_OK)
//	{
//		return ret;
//	}
//
//	// store value in C1 MSB first !
//	*xset = ( (rawdata[0] <<10) | (rawdata[1]<<2)|((rawdata[6]&&(0xc0))>>6));
//	*yset = ( (rawdata[2] <<10) | (rawdata[3]<<2)|((rawdata[6]&&(0x30))>>4));
//	*zset = ( (rawdata[4] <<10) | (rawdata[5]<<2)|((rawdata[6]&&(0x0c))>>2));
//	return HAL_OK;
//}

//HAL_StatusTypeDef mmcma_Perform_mag_measurement( MMC5983 *dev ) {
//
//	/* Store number of transaction errors (to be returned at end of function) */
//		uint8_t errNum = 0;
//		HAL_StatusTypeDef status;
//
//	/*
//	 * Read raw values from acceleration registers (x, y, z -> 24 bits each)
//	 */
//	uint8_t regData_send ;
//	uint8_t regData[7];
//
//	uint8_t regDatacheck[1]={0};
//
//	/*
//		 * Set Take Measearement _Mag to 1 CR0  (DATASHEET PAGE 15 )
//	 */
//	regData_send = MAG_MEASUR_START;
//
//		status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR0, &regData_send);
//		errNum += ( status != HAL_OK );
//
//
//		/*
//		 * Read raw values from x,y,z registers (x, y, z -> 18 bits each)
//		 */
//
//
//
//		/*
//			 * Read status register
//			*/
//
//			while (regDatacheck[0] != DATA_MAG_READY)
//			{
//				 status = MMC5983MA_ReadRegister( dev, MMC5983MA_REG_STATUS, regDatacheck );
//			}
//			errNum += ( status != HAL_OK );
//
//
//
//	 status = MMC5983MA_ReadRegisters( dev, MMC5983MA_REG_X0OUT, regData, 7 );
//	 errNum += ( status != HAL_OK );
//
//	 memcpy(dev->MmcMag, regData, sizeof((dev->MmcMag));
//
//
////	/*
////	 * Combine register valeus to give raw (UNSIGNED) magnetometer readings (18 bits each)
////	 */
////
////
////	magRaw[0] = (uint32_t) (((regData[0] << 16) | (regData[1] << 8) |  (regData[6] & 0xC0)) >> 6) & 0x000FFFFF; /* X-axis */
////	magRaw[1] = (uint32_t) (((regData[2] << 16) | (regData[3] << 8) |  (regData[6] & 0x30)) >> 4) & 0x000FFFFF; /* Y-axis */
////	magRaw[2] = (uint32_t) (((regData[4] << 16) | (regData[5] << 8) |  (regData[6] & 0x0C)) >> 2) & 0x000FFFFF; /* Z-axis */
////
////
////	/* Convert  MagRaw to Gauss (given range  of +-8G / sentativity 0.0625mG per bit / Null output field 0G = (131072 digital value)  )
////	 *  */
////	dev->mag_x =magRaw[0];
////	dev->mag_y =magRaw[1];
////	dev->mag_z =magRaw[2];
//
//	return status;
//
//}



//
//
//uint8_t	mmcma_ReadTemperature( MMC5983 *dev ) {
//
//
//
//
//	/* Store number of transaction errors (to be returned at end of function) */
//	uint8_t errNum = 0;
//	HAL_StatusTypeDef status;
//
//	/*
//	 */
//	uint8_t regData_send;
//	uint8_t regData[1]={0};
//	uint8_t tempdata[1]= {0};
//
//	/*
//	 * Set TM_T and INT_Meas_done_enable in CR0 *initiate a temp measurement and enable the int_pin   (DATASHEET PAGE 14:15)
//	 */
//	regData_send = TEMP_MEASUR_START;
//
//	status = MMC5983MA_WriteRegister( dev, MMC5983MA_REG_ICR0, &regData_send);
//	errNum += ( status != HAL_OK );
//
//	/*
//	 * Read status register
//	*/
//
//	while (regData[0] != DATA_TEMP_READY)
//	{
//		 status = MMC5983MA_ReadRegister( dev, MMC5983MA_REG_STATUS, regData );
//	}
//	errNum += ( status != HAL_OK );
//	/*
//	 * Read raw values from temperature register TOUT 8 bit
//	 */
//
//	 status = MMC5983MA_ReadRegister( dev, MMC5983MA_REG_TOUT, tempdata );
//	 errNum += ( status != HAL_OK );
//
//
//	 /*
//	 * Convert to deg C (0.8degC/LSB, range -75:125  -75 = 0000 0000   * no sign bit ?)
//	 */
//
//		memcpy(dev->MmcTemp, tempdata, sizeof((dev->MmcTemp);
//
//	return errNum;
//
//}






//////


/*
HAL_StatusTypeDef ms5839_get_u_adc(uint8_t address, uint8_t osr,uint8_t adc_read_cmd, uint32_t *adcdata)
{
	HAL_StatusTypeDef ret;
	uint8_t addata[3];

	// Send configuration register data
	ret = HAL_I2C_Master_Transmit(&hi2c1,address, &osr, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK)
	{
		return ret;
	}

HAL_Delay(100);
// Send configuration register data
	ret = HAL_I2C_Master_Transmit(&hi2c1,address, &adc_read_cmd, 1, 50);
		if(ret != HAL_OK)
		{
			return ret;
		}

	// Receive voltage data, two bytes
	ret = HAL_I2C_Master_Receive(&hi2c1,(address|0x01), addata, 3, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	// Assemble adc reading data from two bytes
	*adcdata = (((addata[0]<<16) | (addata[1]<<8)|(addata[2]))&(0x0FFF));
	return HAL_OK;
}*/
/*
void Calculate_Temperature_MS5839(uint16_t c5, uint16_t c6, uint32_t d2, int32_t *temp)
{	int32_t dt=0;
	dt =(int32_t)d2-((int32_t)c5<<8);
	*temp = 2000 +((int64_t)dt *((int64_t)c6>>23);


}*/

	/////////////////////////////
//	HAL_StatusTypeDef ms5839_calculate_temperature_and_pressure(uint16_t c1, uint16_t c2, uint16_t c3, uint16_t c4,uint16_t c5,uint16_t c6, uint32_t d1,uint32_t d2, float *temperature, float *pressure)
//	{
//			HAL_StatusTypeDef ret;
//		int32_t dt, temp;
//		int64_t off, sens, p, temp2,sens2, off2,ti,offi,sensi;
//
//
//		// Difference between actual and reference temperature = D2 - Tref
//			dt =(int32_t)d2-((int32_t)c5<<8);
//		// Acual temperature = 2000 + dT * TEMPSENS
//
//			temp = 2000 +(int64_t)dt *((int64_t)c6>>23);
//				// Second order temperature compensation
//			if( temp >2000 ){//high temperature
//				ti =0;
//				offi = 0 ;
//				sensi = 0;
//				}
//			else if( temp > 1000 & temp<= 2000 ){
//				//low temp
//				ti=(12*(int64_t) (dt<<2))>>35;
//				offi=(30*(int64_t)((temp -2000)<<2))>>8;
//				sensi=0;
//				}
//			else{
//				//very low
//				ti=(12*(int64_t) (dt<<2))>>35;
//				offi=(30*(int64_t)((temp -2000)<<2))>>8;
//				sensi=0;
//				}
//		temp2=((int64_t)(temp) -(int64_t)(ti))/100;
//		off2=((int64_t)(off) -(int64_t)(offi));
//		sens2=((int64_t)(sens) -(int64_t)(sensi));
//		// OFF = OFF_T1 + TCO * dT
//		off += off2 ;
//		off = ( (int64_t)(c2) << 17 ) + ( ( (int64_t)(c4) * dt ) >> 6 ) ;
//
//		//
//
//		// Sensitivity at actual temperature = SENS_T1 + TCS * dT
//		sens += sens2 ;
//		sens = ( (int64_t)c1 << 16 ) + ( ((int64_t)c3 * dt) >> 7 ) ;
//
//
//		// Temperature compensated pressure = D1 * SENS - OFF
//	p = ( ( (d1 * sens) >> 21 ) - off ) >> 15 ;
////	temperature = ( (float)temp ) / 100;
//	*temperature = ( (float)temp - temp2 ) / 100;
//		*pressure = (float)p / 100;
//
//		return HAL_OK;
//	}
//
//	HAL_StatusTypeDef ms5839_get_u_adc(uint8_t address, uint8_t osr_d2,uint8_t adc_read_cmd, uint64_t *adcdata)
//	{
//		HAL_StatusTypeDef ret;
//		uint8_t addata[3];
//
//
//		// Send configuration register data
//		ret = HAL_I2C_Master_Transmit(&hi2c1,address, &osr_d2, 1, 50);
//		if(ret != HAL_OK)
//		{
//			return ret;
//		}
//
//	HAL_Delay(100);
//	// Send configuration register data
//		ret = HAL_I2C_Master_Transmit(&hi2c1,address, &adc_read_cmd, 1, 50);
//			if(ret != HAL_OK)
//			{
//				return ret;
//			}
//
//		// Receive voltage data, two bytes
//		ret = HAL_I2C_Master_Receive(&hi2c1,(address|0x01), addata, 3, 50);
//		if(ret != HAL_OK)
//		{
//			return ret;
//		}
//
//		// Assemble adc reading data from two bytes
//
//
//		*adcdata = ((addata[0]<<16) | (addata[1]<<8)| (addata[2]) & (0x0FFF));
//		return HAL_OK;
//	}

/*
 * I2C FUNCTIONS
 */

HAL_StatusTypeDef MMC5983MA_ReadRegister( MMC5983 *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, MMC5983MA_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MMC5983MA_ReadRegisters( MMC5983 *dev, uint8_t reg, uint8_t *data, uint8_t length ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, MMC5983MA_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef MMC5983MA_WriteRegister( MMC5983 *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Write( dev->i2cHandle, MMC5983MA_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}


