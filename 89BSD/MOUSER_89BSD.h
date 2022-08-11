/*
 * MOUSER_89BSD.h
 *
 *  Created on: Feb 12, 2022
 *      Author: Gaming
 */

#ifndef INC_MOUSER_89BSD_H_
#define INC_MOUSER_89BSD_H_


#include "stm32l4xx_hal.h" /* inc Hal_lib */
#include <stdint.h>
#include <math.h>





/*
 * DEFINES
 */
#define MOUSER_BSD89_I2C_ADDR	(0x77 << 1) /*I2C 7 BITS ADRESS :89BSD CALCULATION METHOD (p. 1) */



/*
/* Register defines */


 /* DEVICE  PROM REGG ADRESS  FROM 0xA0 To 0xAE  for calibration individual adresses
 check  Memorey MAPPING in89BSD CALCULATION METHOD (p. 6) */


#define  MOUSER_BSD89_REG_PROM_START_ADDRESS		0xA0

 /* DEVICE  ADC DATA ADRESS 0x00  24bit data   check  :
  I2C pressure response (D1) on 24 bits from 89BSD  in89BSD CALCULATION METHOD (p. 4) */

#define  MOUSER_BSD89_REG_ADC_READ_DATA_ADDRESS		0x00


 /* DEVICE  CMDS */

#define  MOUSER_BSD89_RESET_CMD		0xE1 			/*DEVICE RESET CMD check in89BSD CALCULATION METHOD (p. 2)  */

/* USER'S DEVICE  START CODE */

#define  MOUSER_BSD89_START_CODE    0xB8
#define  MOUSER_BSD89_HEADER_START_CODE    0xB8

 /* DEVICE WAIT FOR CONVERSION TIME  check table 2 Conversion Time and table 3 Resolution  in89BSD DATASHEET (p. 4) */

#define  MOUSSER_WAIT_TIME_OSR_256                1
#define  MOUSSER_WAIT_TIME_OSR_512                2
#define  MOUSSER_WAIT_TIME_OSR_1024               3
#define  MOUSSER_WAIT_TIME_OSR_2048               5
#define  MOUSSER_WAIT_TIME_OSR_4096               10



typedef enum {
	CONVERT_D1_OSR_256                    		= 0x40,
	CONVERT_D1_OSR_512                    		= 0x42,
	CONVERT_D1_OSR_1024                    		= 0x44,
	CONVERT_D1_OSR_2048                    		= 0x46,
	CONVERT_D1_OSR_4096                    		= 0x48,

} mouser_89bsd_osr_mode_pres;


typedef enum {
  CONVERT_D2_OSR_256                    		= 0x50,
  CONVERT_D2_OSR_512                    		= 0x52,
  CONVERT_D2_OSR_1024                    		= 0x54,
  CONVERT_D2_OSR_2048                    		= 0x56,
  CONVERT_D2_OSR_4096                    		= 0x58,

} mouser_89bsd_osr_mode_temp;


typedef struct {
	mouser_89bsd_osr_mode_pres  osr_pressure;
	mouser_89bsd_osr_mode_temp  osr_temperature;

}conversion_osr;

//typedef union{
//	conversion_osr                   press_mode;
//	conversion_osr                temp_mode;
//
//} conversion_mode;

typedef struct __attribute__ ((packed)){
  uint8_t headerStartCode;
  uint8_t chipID;
  /* Calibration values */

  uint8_t factory_calibration[14];

  uint8_t osrP; //
  uint8_t osrT;
}TE89BSD_header;

typedef struct __attribute__ ((packed)){

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	//conversion_osr  *convert;
	  uint8_t dataStartCode; // = 0xAA;
	/* Sensor Flag */

	  uint8_t IsReadingTE89BSDFlag;

	/* Calibration values */
	//uint8_t factory_calibration[14];

	/* 24bit pressure (MSB) bytes handler */
	uint8_t Te89Press[3];

	/* 24bit temperature (MSB) bytes handler  */
	uint8_t Te89Temp[3];


} TE89BSD ; //   REMAINING TASK !!CHECK THE DATASHEET FOR THE CONVERSION OSR RETURN FUNCTION  & RENAME FUNCTIONS

typedef struct __attribute__ ((packed)){
 	  uint32_t m_seconds;
 	  uint16_t u_seconds;
}TE89BSD_TS;


/*
 *
 RESET *
 */

HAL_StatusTypeDef TE89BSDHardreset(TE89BSD *dev, I2C_HandleTypeDef *i2cHandle  );

/*
 *
 * INITIALISATION
 *
 */
HAL_StatusTypeDef TE89BSDInitialise( TE89BSD *dev, I2C_HandleTypeDef *i2cHandle ,TE89BSD_header *header );

/*
 *
 * TEMPERATURE AND PRESSURE READ
 *
 */
uint8_t TE89BSDReadPressure( TE89BSD *dev ,TE89BSD_header*header) ;
uint8_t TE89BSDReadTemperature( TE89BSD *dev ,TE89BSD_header*header);

/*
 *
 * TEMPERATURE AND PRESSURE READ (DMA)
 *
 */
HAL_StatusTypeDef TE89BSD_pressure_DMA( TE89BSD *dev ,TE89BSD_header *header) ;
HAL_StatusTypeDef TE89BSD_temperature_DMA( TE89BSD *dev,TE89BSD_header *header) ;
void  TE89BSD_pressure_ReadDMA_Complete( TE89BSD *dev );


/*
	 * I2C_ FUNCTIONS
	 */
	HAL_StatusTypeDef TE89BSD_ReadRegister(  TE89BSD *dev, uint8_t reg, uint8_t *data );
	HAL_StatusTypeDef TE89BSD_ReadRegisters( TE89BSD *dev, uint8_t reg, uint8_t *data, uint8_t length );

	HAL_StatusTypeDef TE89BSD_WriteRegister( TE89BSD *dev, uint8_t reg, uint8_t *data );
	HAL_StatusTypeDef TE89BSD_WriteCmd( TE89BSD *dev, uint8_t *data );


/*
	 *
	 * I2C_DMA
*/
	HAL_StatusTypeDef TE89BSD_WriteCmd_DMA( TE89BSD *dev, uint8_t *data );
	HAL_StatusTypeDef TE89BSD_ReadRegisters_DMA( TE89BSD *dev, uint8_t reg, uint8_t *data, uint8_t length );


#endif /* INC_MOUSER_89BSD_H_ */
