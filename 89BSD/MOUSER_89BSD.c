/*
 * MOUSER_89BSD.c
 *
 *  Created on: Feb 12, 2022
 *      Author: Gaming
 */
#include "MOUSER_89BSD.h"
#include <string.h>


/*
 *
 HARDRESET *
 */

HAL_StatusTypeDef TE89BSDHardreset( TE89BSD *dev, I2C_HandleTypeDef *i2cHandle)
{


	/* init struct parameters */
	dev->i2cHandle 		= i2cHandle;

	/* Store number of transaction errors (to be returned at end of function) */

		HAL_StatusTypeDef status;
		uint8_t regData;

		regData= MOUSER_BSD89_RESET_CMD; /* initialise the regData with The hard reset cmd  0xE1 in89BSD CALCULATION METHOD (p. 2) **/
		status =TE89BSD_WriteCmd( dev , &regData);


		HAL_Delay(200);

		return status;
	}

/*
 *
 * INITIALISATION
 *
 */


HAL_StatusTypeDef TE89BSDInitialise( TE89BSD *dev, I2C_HandleTypeDef *i2cHandle ,TE89BSD_header *header) {

	/* init struct parameters */
	dev->i2cHandle 		= i2cHandle;

	/*header struct  initalization  */
	memset(header->headerStartCode, MOUSER_BSD89_HEADER_START_CODE, sizeof(header->headerStartCode));
	memset(header->factory_calibration, 0, sizeof(header->factory_calibration));


	/* Data sturct initilisation  */
	memset(dev->dataStartCode, MOUSER_BSD89_START_CODE,sizeof( header->headerStartCode));
	memset(dev->Te89Press, 0, 3);
	memset(dev->Te89Temp, 0, 3);





	/* Store number of transaction errors (to be returned at end of function) */
	HAL_StatusTypeDef status;

	/*
			 * read  14 bytes Calibration Values   DEVICE  PROM REGG ADRESS  FROM 0xA0 To 0xAE  for calibration individual adresses
 check  Memorey MAPPING in89BSD CALCULATION METHOD (p. 6) *
		 */
			uint8_t regData[14];

			 status = TE89BSD_ReadRegisters( dev, MOUSER_BSD89_REG_PROM_START_ADDRESS, regData, 14 );
			 if (status== HAL_OK)
			 {
			memcpy(header->factory_calibration, regData, sizeof(header->factory_calibration));
			 }
			return status;
	}




/*
 *
 * TEMPERATURE AND PRESSURE READ
 *
 */

uint8_t TE89BSDReadPressure( TE89BSD *dev ,TE89BSD_header*header) {

	/* init the  Pressure Oversampling rate  check table 2 Conversion Time and table 3 Resolution  in89BSD DATASHEET (p. 4) */

		memset(header->osrP,CONVERT_D1_OSR_1024, sizeof(header->osrP));







	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t cmdData;
	uint8_t regData[3];

	cmdData= header->osrP; /* initialise the CmdData with osr **/


	status =TE89BSD_WriteCmd( dev , &cmdData);


	errNum += ( status != HAL_OK );


		  		  switch (cmdData)
		  		  {
		  		  	  case (CONVERT_D1_OSR_256):
					HAL_Delay(MOUSSER_WAIT_TIME_OSR_256);
		  		  	cmdData =0;
		  		  	break;

		  			  case (CONVERT_D1_OSR_512):
					HAL_Delay(MOUSSER_WAIT_TIME_OSR_512);
		  			cmdData =0;

	  			  	break;

		  			  case (CONVERT_D1_OSR_1024):
					HAL_Delay(MOUSSER_WAIT_TIME_OSR_1024);
		  			cmdData =0;

		  			break;

		  			  case (CONVERT_D1_OSR_2048):
					HAL_Delay(MOUSSER_WAIT_TIME_OSR_2048);
		  			cmdData =0;


	  			    break;

		  			  case (CONVERT_D1_OSR_4096):
					HAL_Delay(MOUSSER_WAIT_TIME_OSR_4096);
		  			cmdData =0;


	  			  	break;


		  			  default :
		  			HAL_Delay(10);
		  			cmdData =0;
		  				  break;
		  		  }

	status = TE89BSD_ReadRegisters( dev, MOUSER_BSD89_REG_ADC_READ_DATA_ADDRESS, regData, 3 );
	errNum += ( status != HAL_OK );

	memcpy(dev->Te89Press, regData, sizeof(regData));
	/* Return number of errors (0 if successful initialisation) */
	return errNum;

}





uint8_t TE89BSDReadTemperature( TE89BSD *dev ,TE89BSD_header*header) {


	/* init the  Pressure Oversampling rate  check table 2 Conversion Time and table 3 Resolution  in89BSD DATASHEET (p. 4) */

		memset(header->osrT, CONVERT_D2_OSR_1024, sizeof(header->osrT));
	/* Store number of transaction errors (to be returned at end of function) */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	uint8_t cmdData;
	uint8_t regData[3];

	cmdData= header->osrT; /* initialise the CmdData with osr **/
	status =TE89BSD_WriteCmd( dev , &cmdData);

	errNum += ( status != HAL_OK );
	  switch (cmdData)
			  		  {
			  		  	  case (CONVERT_D2_OSR_256):
						HAL_Delay(MOUSSER_WAIT_TIME_OSR_256);
			  		  	cmdData =0;
			  		  	break;

			  			  case (CONVERT_D2_OSR_512):
						HAL_Delay(MOUSSER_WAIT_TIME_OSR_512);
			  			cmdData =0;

		  			  	break;

			  			  case (CONVERT_D2_OSR_1024):
						HAL_Delay(MOUSSER_WAIT_TIME_OSR_1024);
			  			cmdData =0;

			  			break;

			  			  case (CONVERT_D2_OSR_2048):
						HAL_Delay(MOUSSER_WAIT_TIME_OSR_2048);
			  			cmdData =0;


		  			    break;

			  			  case (CONVERT_D2_OSR_4096):
						HAL_Delay(MOUSSER_WAIT_TIME_OSR_4096);
			  			cmdData =0;


		  			  	break;


			  			  default :
			  				HAL_Delay(10);
			  				cmdData =0;
			  				  break;
			  		  }

	status = TE89BSD_ReadRegisters( dev, MOUSER_BSD89_REG_ADC_READ_DATA_ADDRESS, regData, 3 );
	errNum += ( status != HAL_OK );


	memcpy(dev->Te89Temp, regData, sizeof(regData));
	/* Return number of errors (0 if successful initialisation) */
	return errNum;

	}


/*
 *
 * TEMPERATURE AND PRESSURE READ (DMA)
 *
 */

HAL_StatusTypeDef TE89BSD_pressure_DMA( TE89BSD *dev ,TE89BSD_header *header){

	/* init the  Pressure Oversampling rate  check table 2 Conversion Time and table 3 Resolution  in89BSD DATASHEET (p. 4) */

		memset(header->osrP,CONVERT_D1_OSR_1024, sizeof(header->osrP));


		/* Store number of transaction errors (to be returned at end of function) */
		uint8_t errNum = 0;
		HAL_StatusTypeDef status;
		uint8_t cmdData;
		uint8_t regData[3];


		cmdData= header->osrP; /* initialise the CmdData with osr **/

		status =TE89BSD_WriteCmd_DMA( dev , &cmdData);
		errNum += ( status != HAL_OK );


				  		  switch (cmdData)
				  		  {
				  		  	  case (CONVERT_D1_OSR_256):
							HAL_Delay(MOUSSER_WAIT_TIME_OSR_256);
				  		  	cmdData =0;
				  		  	break;

				  			  case (CONVERT_D1_OSR_512):
							HAL_Delay(MOUSSER_WAIT_TIME_OSR_512);
				  			cmdData =0;

			  			  	break;

				  			  case (CONVERT_D1_OSR_1024):
							HAL_Delay(MOUSSER_WAIT_TIME_OSR_1024);
				  			cmdData =0;

				  			break;

				  			  case (CONVERT_D1_OSR_2048):
							HAL_Delay(MOUSSER_WAIT_TIME_OSR_2048);
				  			cmdData =0;


			  			    break;

				  			  case (CONVERT_D1_OSR_4096):
							HAL_Delay(MOUSSER_WAIT_TIME_OSR_4096);
				  			cmdData =0;


			  			  	break;


				  			  default :
				  			HAL_Delay(10);
				  			cmdData =0;
				  				  break;
				  		  }

			status = TE89BSD_ReadRegisters_DMA(dev, MOUSER_BSD89_REG_ADC_READ_DATA_ADDRESS, regData, 3 );
			errNum += ( status != HAL_OK );
			memcpy(dev->Te89Press, regData, sizeof(regData));

			/* Return number of errors (0 if successful initialisation) */
			return errNum;


}



void TE89BSD_pressure_ReadDMA_Complete( TE89BSD *dev ){


	/* TO EXCUTE IN I2C RX CALLBACK FUNCTION */
}


/*
 *
 *
 *
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef TE89BSD_ReadRegister( TE89BSD *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, MOUSER_BSD89_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}

HAL_StatusTypeDef TE89BSD_ReadRegisters( TE89BSD *dev, uint8_t reg, uint8_t *data, uint8_t length ) {

	return HAL_I2C_Mem_Read( dev->i2cHandle, MOUSER_BSD89_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );

}

HAL_StatusTypeDef TE89BSD_WriteRegister( TE89BSD *dev, uint8_t reg, uint8_t *data ) {

	return HAL_I2C_Mem_Write( dev->i2cHandle, MOUSER_BSD89_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );

}
HAL_StatusTypeDef TE89BSD_WriteCmd( TE89BSD *dev, uint8_t *data ) {

	return HAL_I2C_Master_Transmit(dev->i2cHandle, MOUSER_BSD89_I2C_ADDR, data, I2C_MEMADD_SIZE_8BIT, HAL_MAX_DELAY);
}

/*
	 *
	 * I2C_DMA
*/

HAL_StatusTypeDef TE89BSD_WriteCmd_DMA( TE89BSD *dev, uint8_t *data ){

	return HAL_I2C_Master_Transmit_DMA(dev->i2cHandle, MOUSER_BSD89_I2C_ADDR, data, I2C_MEMADD_SIZE_8BIT);
}


HAL_StatusTypeDef TE89BSD_ReadRegisters_DMA( TE89BSD *dev, uint8_t reg, uint8_t *data, uint8_t length ){

	return HAL_I2C_Mem_Read_DMA( dev->i2cHandle, MOUSER_BSD89_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length);

}
