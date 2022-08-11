/*
 * MMC5983MA.h
 *
 *  Created on: Nov 10, 2021
 *      Author: Gaming
 */

#ifndef INC_MMC5983MA_H_
#define INC_MMC5983MA_H_
#include "stm32l4xx_hal.h" /* inc Hal_lib */
#include "main.h"

/*
 * DEFINES
 */
#define MMC5983MA_I2C_ADDR	(0x30<<1) /*I2C 7 BITS ADRESS Data transfer DATASHEET (p. 17) */
#define MMC5983MA_DEVICE_ID	0x30   /*DEVICE ID  -READ_ONLY DEFINED BY FACTORY :USE TO CHECK THE SENSOR -*/

/*
 * REGISTERS MAP DATASHEET(p. 13)  *PS: ALL REGISTERS ARE 8 BITS
 */
#define MMC5983MA_REG_X0OUT			0x00 	/* X[17:10]*/
#define MMC5983MA_REG_X1OUT			0x01	/* X[9:2]*/
#define MMC5983MA_REG_Y0OUT			0x02
#define MMC5983MA_REG_Y1OUT			0x03
#define MMC5983MA_REG_Z0OUT			0x04
#define MMC5983MA_REG_Z1OUT			0x05
#define MMC5983MA_REG_XYZ2OUT		0x06	/* USED FOR TO COMBIME 18 BIT DATA  X[1:0]*/
#define MMC5983MA_REG_TOUT			0x07	/* TEMP OUTPUT*/
#define MMC5983MA_REG_STATUS		0x08
#define MMC5983MA_REG_ICR0			0x09	/* INTERNEL CONTROL REGS*/
#define MMC5983MA_REG_ICR1			0x0A
#define MMC5983MA_REG_ICR2			0x0B
#define MMC5983MA_REG_ICR3			0x0C
#define MMC5983MA_REG_ID			0x2F	/* DIVICE ID REG  FACTORY DEF READ ONLY*/

/*Sample rates determine how often the chip take measurements in continuous mode IN CR2[2:0] **ENABLE BIT IS [3] DATASHEET(p. 16)*/
#define  MODR_ONESHOT  				0x00
#define  MODR_1Hz 					0x09
#define  MODR_10Hz  				0x0A
#define  MODR_20Hz  				0x0B
#define  MODR_50Hz  				0x0C
#define	 MODR_100Hz  				0x0D
#define	 MODR_200Hz  				0x0E 	/*BW = 0x01 only */
#define	 MODR_1000Hz 				0x0F	/* BW = 0x11 only */

/*Bandwidth selection bits in CR1 [1:0]they control the duration of each measurements  DATASHEET(p. 15)
 * check specifications for noise annd current consumption   DATASHEET(p. 2)  */
#define	 MBW_100Hz 					0x00  // 8 ms measurement time
#define  MBW_200Hz 					0x01  // 4 ms
#define  MBW_400Hz					0x02  // 2 ms
#define  MBW_800Hz 					0x03  // 0.5 ms


/* the sensor use Set/Reset functions to remove the offset by sending  largue amout of current in both directions those bits
 * determine how often the set/reset in continuous mode IN CR2 [6:4] **ENABLE BIT IS[7] DATASHEET(p. 16)*/

#define   SET_1     				0x00 // Set/Reset each data measurement
#define   MSET_25   	 			0x90 // each 25 data measurements
#define   MSET_75   				0xA0
#define   MSET_100   				0xB0
#define   MSET_250   				0xC0
#define   MSET_500   				0xD0
#define   MSET_1000  				0xE0
#define   MSET_2000  				0xF0

/*cmds */

#define 	HARD_RESET_CMD 			0x80 	/* IN CR1 */
#define 	CONFIG_CMD 				0x0D  	/*IN CR2 (** 100HZ& NO AUTO SET) SET THE DATA RATE IN CONTINUOUS MODE AND THE AUTO SET/RESET*/
#define 	SET_ENABLE 				0x02	/*IN CR3 */
#define 	RESET_ENABLE 			0x04	/*IN CR3 */
#define 	SET_START 				0x08	/* IN CR0*/
#define 	RESET_START 			0x10	/* IN CR0*/

#define  	DATA_MAG_READY 			0x11
#define  	DATA_TEMP_READY 		0x12

#define  	MEASUR_CMD 				0x00	/* IN CR1*/
#define  	MAG_MEASUR_START 		0x05	/* IN CR0*/
#define  	TEMP_MEASUR_START 		0x06	/*IN CR0*/


typedef struct __attribute__ ((packed)){
  uint8_t headerStartCode;// = 0xc0;
  uint8_t chipID;

  uint8_t status;
  float odr; //sampling frequency in Hz
  float range;
  /*******/
  uint8_t  SetField[7];
  uint8_t ResetField[7];


  /*uint16_t setFieldX;
  uint16_t setFieldZ;
  uint16_t setFieldY;
  uint16_t resetFieldX;
  uint16_t resetFieldZ;
  uint16_t resetFieldY;
*/
}MMC_header;

typedef struct __attribute__ ((packed)){

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
	uint8_t dataStartCode; // = 0xcc;
	uint8_t IsReadingMmcFlag;

	/* angular rate  (X, Y, Z) in Gauss */
	uint8_t MmcMag[7];
	/* Temperature data in deg */
	uint8_t MmcTemp;

} MMC5983;
typedef struct __attribute__ ((packed)){
 	  uint32_t m_seconds;
 	  uint16_t u_seconds;
}MMC5983_TS ;


//typedef struct {
//
//	/* I2C handle */
//	I2C_HandleTypeDef *i2cHandle;
//
//	/* magnetic field data  */
//
//
//uint8_t StartCode[1]; // = 0xcc;
//uint8_t MmcMag[7];
//
//	/* Temperature data  */
//uint8_t MmcTemp[1];
//
//} MMC5983;

//
//typedef struct mmc_data {
//  uint8_t StartCode;// = 0xcc;
//  uint32_t ts;
//  uint8_t data[7];
//}mmc_data;


//typedef struct mmc_header {
//  uint8_t startcode;// = ;
//  uint8_t id;
//  timestamp ts;
//  uint8_t status;
//  uint16_t odr;
//  uint16_t setFieldX;
//  uint16_t setFieldZ;
//  uint16_t setFieldY;
//  uint16_t resetFieldX;
//  uint16_t resetFieldZ;
//  uint16_t resetFieldY;
//}mmc_header;
// write 0x80 to CR1


HAL_StatusTypeDef MmcHardReset(MMC5983 *dev, I2C_HandleTypeDef *i2cHandle);


/* INITIALISATION ROUTINE */
uint8_t MmcCheckID(MMC5983 *dev, I2C_HandleTypeDef *i2cHandle , MMC_header *header );



uint8_t MmcInitialiseOneShot( MMC5983 *dev, I2C_HandleTypeDef *i2cHandle, MMC_header *header );


uint8_t MmcConfigSet( MMC5983 *dev );


HAL_StatusTypeDef    MmcGetSetField( MMC5983 *dev , MMC_header *header);



uint8_t MmcConfigReset( MMC5983 *dev );

HAL_StatusTypeDef    MmcGetResetField( MMC5983 *dev ,MMC_header *header);


/* CONTINUOUS MODE ROUTINE */


uint8_t MmcConfigContiniousINT( MMC5983 *dev ,MMC_header *header);
//HAL_StatusTypeDef check_for_mag_measurement(uint8_t MMC5983MA_ADDRESS, uint8_t MMC5983MA_STATUS,uint8_t  data_ready);
//HAL_StatusTypeDef  Perform_mag_x_measurement(uint8_t MMC5983MA_ADDRESS,uint8_t MMC5983MA_XOUT_0,uint8_t  MMC5983MA_XYZOUT_2 ,uint32_t *x_set);



HAL_StatusTypeDef    MmcReadMagField( MMC5983 *dev );
HAL_StatusTypeDef	MmcReadTemperature( MMC5983 *dev ) ;

//
////uint8_t	mmcma_ReadTemperature( MMC5983 *dev ) ;
//HAL_StatusTypeDef  calculalte_offset(float *offset);

/*
 * I2C_ FUNCTIONS
 */
HAL_StatusTypeDef MMC5983MA_ReadRegister(  MMC5983 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef MMC5983MA_ReadRegisters( MMC5983 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef MMC5983MA_WriteRegister( MMC5983 *dev, uint8_t reg, uint8_t *data );
#endif /* INC_MMC5983MA_H_ */
