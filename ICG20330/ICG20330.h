/*
 * ICG20330.h
 *
 *  Created on: Nov 15, 2021
 *      Author: Gaming
 */

#ifndef INC_ICG20330_H_
#define INC_ICG20330_H_

#include "stm32l4xx_hal.h" /* inc Hal_lib */

/*
 * DEFINES
 */
#define ICG20330_I2C_ADDR	(0x68<<1) /*I2C 7 BITS ADRESS  =b110100X  x=LSB determined by SA0 PIN  SA0 TO GND  DATASHEET (p. 24) */
#define ICG20330_DEVICE_ID	0x92   /*DEVICE ID  -WHO IM REGISTER VALUE (adress = 0x75)-*/

/*
 * REGISTERS MAP DATASHEET(p. 35-36)  *PS: ALL REGISTERS ARE 8 BITS
 */
#define ICG20330_REG_SELF_TEST_X_GYRO					0x00 	/* X_SELF_TEST REG[7:0] DATASHEET(p. 37)*/
#define ICG20330_REG_SELF_TEST_Y_GYRO					0x01	/* Y_SELF_TEST REG[7:0]*/
#define ICG20330_REG_SELF_TEST_Z_GYRO					0x02	/* Z_SELF_TEST REG[7:0]*/
/* not  #define IN DATASHEET				0x03*/
#define ICG20330_REG_XG_OFFS_TC_H						0x04   /* XG_OFFSET TEMP COMPENSATION REG[9:8]	DATASHEET(p. 38-40)*/
#define ICG20330_REG_XG_OFFS_TC_L						0x05	/* XG_OFFSET TEMP COMPENSATION REG[7:0]*/
/*  not  #define IN DATASHEET				0x06*/
#define ICG20330_REG_YG_OFFS_TC_H						0x07	/* YG_OFFSET TEMP COMPENSATION REG[9:8]*/
#define ICG20330_REG_YG_OFFS_TC_L						0x08	/* YG_OFFSET TEMP COMPENSATION REG[7:0]*/
/* not  #define IN DATASHEET					0x09	/* INTERNEL CONTROL REGS*/
#define ICG20330_REG_ZG_OFFS_TC_H						0x0A	/* ZG_OFFSET TEMP COMPENSATION REG[9:8]*/
#define ICG20330_REG_ZG_OFFS_TC_L						0x0B	/* ZG_OFFSET TEMP COMPENSATION REG[7:0]*/
/*
 * **
 */

#define ICG20330_REG_XG_OFFS_USRH						0x13 	/* X USER DEFINED OFFSET [15:8]	  DATASHEET(p. 40-42)*/
#define ICG20330_REG_XG_OFFS_USRL						0x14	/* X USER DEFINED OFFSET [7:0]*/

#define ICG20330_REG_YG_OFFS_USRH						0x15	/* Y USER DEFINED OFFSET [15:8]*/
#define ICG20330_REG_YG_OFFS_USRL						0x16	/* Y USER DEFINED OFFSET [7:0]*/

#define ICG20330_REG_ZG_OFFS_USRH						0x17	/* Z USER DEFINED OFFSET [15:8]*/
#define ICG20330_REG_ZG_OFFS_USRL						0x18	/* Z USER DEFINED OFFSET [7:0]*/

#define ICG20330_REG_SMPLRT_DIV							0x19	/* SAMPLE RATE DIV reg DATASHEET(p.42)*/
																/* SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) Where INTERNAL_SAMPLE_RATE = 1kHz */

#define ICG20330_REG_CONFIG								0x1A	/* CONFIG reg DATASHEET(p.43) **fifo enable **external snc **DLPF filte see table */
#define ICG20330_REG_GYRO_CONFIG						0x1B	/* GYRO_CONFIG reg DATASHEET(p.44) **full scale select **FBCHOISE for filter */

/*
 * **
 */
#define ICG20330_REG_ICG20330_FIFO_EN					0x23	/* FIFO enable reg for x,y,z and temp DATASHEET(p.45) */
#define ICG20330_REG_FSYNC_INT							0x36	/*  FSYNC INTERRUPT STATUS DATASHEET(p.46)*/
#define ICG20330_REG_INT_PIN_CFG						0x37	/*  INT PIN / BYPASS ENABLE CONFIGURATION  DATASHEET(p.46)*/
#define ICG20330_REG_INT_ENABLE							0x38	/* INTERRUPT ENABLE DATASHEET(p.47)*/
#define ICG20330_REG_INT_STATUS							0x3A	/* INTERRUPT STATUS DATASHEET(p.47)*/
/*TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 25degC */
#define ICG20330_REG_TEMP_OUT_H							0x41	/* TEMP OUT Reg [15:8] DATASHEET(p.48)*/
#define ICG20330_REG_TEMP_OUT_L							0x42	/* TEMP OUT Reg [7:0] */

/*GYRO_XOUT = Gyro_Sensitivity * X_angular_rate */
/* Nominal Conditions FS_SEL = 0 Gyro_Sensitivity = 131 LSB/(º/s)*/
#define ICG20330_REG_GYRO_XOUT_H						0x43	/*  X OUT [15:8]DATASHEET(p.49)*/
#define ICG20330_REG_GYRO_XOUT_L						0x44	/*  X OUT [7:0]*/



#define ICG20330_REG_GYRO_YOUT_H						0x45	/* Y OUT [15:8]*/
#define ICG20330_REG_GYRO_YOUT_L						0x46	/* Y OUT [7:0]*/

#define ICG20330_REG_GYRO_ZOUT_H						0x47	/* Z OUT [15:8]*/
#define ICG20330_REG_GYRO_ZOUT_L						0x48	/* Z OUT [7:0]*/

#define ICG20330_REG_SIGNAL_PATH_RESET					0x68	/* SIGNAL PATH RESET*/
#define ICG20330_REG_USER_CTRL							0x6A	/* USER CONTROL DATASHEET(p.51)*/
#define ICG20330_REG_PWR_MGMT_1							0x6B	/*  POWER MANAGEMENT 1DATASHEET(p.52)*/
#define ICG20330_REG_PWR_MGMT_2							0x6C	/*  POWER MANAGEMENT 2DATASHEET(p.53)*/
#define ICG20330_REG_FIFO_COUNTH						0x72	/* FIFO COUNT REGISTER DATASHEET(p.53)*/
#define ICG20330_REG_FIFO_COUNTL						0x73	/* FIFO COUNT REGISTER DATASHEET(p.53)*/
#define ICG20330_REG_FIFO_R_W							0x74	/*  FIFO READ WRITE DATASHEET(p.54) */
#define ICG20330_REG_WHO_AM_I							0x75	/* WHO_AM_I reg DATASHEET(p.54)*/

///*Sample rates determine how often the chip take measurements in continuous mode IN CR2[2:0] **ENABLE BIT IS [3] DATASHEET(p. 16)*/
//#define  MODR_ONESHOT  				0x00
//#define  MODR_1Hz 					0x09
//#define  MODR_10Hz  				0x0A
//#define  MODR_20Hz  				0x0B
//#define  MODR_50Hz  				0x0C
//#define	 MODR_100Hz  				0x0D
//#define	 MODR_200Hz  				0x0E 	/*BW = 0x01 only */
//#define	 MODR_1000Hz 				0x0F	/* BW = 0x11 only */
//
///*Bandwidth selection bits in CR1 [1:0]they control the duration of each measurements  DATASHEET(p. 15)
// * check specifications for noise annd current consumption   DATASHEET(p. 2)  */
//#define	 MBW_100Hz 					0x00  // 8 ms measurement time
//#define  MBW_200Hz 					0x01  // 4 ms
//#define  MBW_400Hz					0x02  // 2 ms
//#define  MBW_800Hz 					0x03  // 0.5 ms
//
//
///* the sensor use Set/Reset functions to remove the offset by sending  largue amout of current in both directions those bits
// * determine how often the set/reset in continuous mode IN CR2 [6:4] **ENABLE BIT IS[7] DATASHEET(p. 16)*/
//
//#define   SET_1     				0x00 // Set/Reset each data measurement
//#define   MSET_25   	 			0x90 // each 25 data measurements
//#define   MSET_75   				0xA0
//#define   MSET_100   				0xB0
//#define   MSET_250   				0xC0
//#define   MSET_500   				0xD0
//#define   MSET_1000  				0xE0
//#define   MSET_2000  				0xF0
//
///*cmds */
//
//#define 	HARD_RESET_CMD 			0x80 	/* IN CR1 */
//#define 	CONFIG_CMD 				0x0D  	/*IN CR2 (** 100HZ& NO AUTO SET) SET THE DATA RATE IN CONTINUOUS MODE AND THE AUTO SET/RESET*/
//#define 	SET_ENABLE 				0x02	/*IN CR3 */
//#define 	RESET_ENABLE 			0x04	/*IN CR3 */
//#define 	SET_START 				0x08	/* IN CR0*/
//#define 	RESET_START 			0x10	/* IN CR0*/
//
//#define  	DATA_MAG_READY 			0x01
//#define  	DATA_TEMP_READY 		0x02
//
//#define  	MEASUR_CMD 				0x00	/* IN CR1*/
//#define  	MAG_MEASUR_START 		0x05	/* IN CR0*/
//#define  	TEMP_MEASUR_START 		0x06	/*IN CR0*/

typedef struct __attribute__ ((packed)){
  uint8_t headerStartCode;
  uint8_t chipID;
   float odr; //sampling frequency in Hz
   float range;
   float lpf;
   float hpf;
}ICG_header;

typedef struct __attribute__ ((packed)){

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
	uint8_t dataStartCode;
	uint8_t IsReadingIcgFlag;
	/* angular rate  (X, Y, Z) in Gauss */

	uint8_t IcgGyro[6];

	/* Temperature data in deg */
	uint8_t IcgTemp[2];

} ICG;

typedef struct __attribute__ ((packed)){
 	  uint32_t m_seconds;
 	  uint16_t u_seconds;
}ICG_TS;
//typedef struct {
//
//
//	/* I2C handle */
//	I2C_HandleTypeDef *i2cHandle;
//	uint8_t startcode;
//	uint16_t gyro[3];
//
//		/* Temperature data in deg */
//	uint16_t temp_C;
//
//
//
//} ICG;


/*
 * INITIALISATION
 */
uint8_t ICG20330_check_id( ICG *dev, I2C_HandleTypeDef *i2cHandle,ICG_header *header );
uint8_t ICG20330_Initialise( ICG *dev, I2C_HandleTypeDef *i2cHandle, ICG_header *header );
HAL_StatusTypeDef ICG20330_ReadTemperature( ICG *dev );
HAL_StatusTypeDef ICG20330_ReadGyro( ICG *dev );


/*
	 * I2C_ FUNCTIONS
	 */
	HAL_StatusTypeDef ICG20330_ReadRegister(  ICG *dev, uint8_t reg, uint8_t *data );
	HAL_StatusTypeDef ICG20330_ReadRegisters( ICG *dev, uint8_t reg, uint8_t *data, uint8_t length );

	HAL_StatusTypeDef ICG20330_WriteRegister( ICG *dev, uint8_t reg, uint8_t *data );


#endif /* INC_ICG20330_H_ */
