/*
 * BMG250.h
 *
 *  Created on: Dec 16, 2021
 *      Author: Gaming
 */

#ifndef INC_BMG250_H_
#define INC_BMG250_H_


#include "stm32l4xx_hal.h" /* inc Hal_lib */

/*
 * DEFINES
 */
#define BMG250_I2C_ADDR	(0x68<<1) /*I2C 7 BITS ADRESS  =b110100X  x=LSB determined by SDO PIN  SDO TO VDDIO=3,3v (**OR MIGRATE TO I2C 2 BMG250 CONFLICT **)   DATASHEET (p. 24) */
#define BMG250_DEVICE_ID	0xD5   /*DEVICE ID  -WHO IM REGISTER VALUE (adress = 0X00)-*/


/* START CODE USER DEFINED  */

#define BMG250_HEADER_START_CODE 			0xB0
#define BMG250_DATA_START_CODE 				0xB5

/*
 * REGISTERS MAP DATASHEET(p. 25)  *PS: ALL REGISTERS ARE 8 BITS
 */
#define BMG250_REG_CHIP_ID					0x00 	/*  DATASHEET(p. 26)*/
/*#define BMG250_REG RESERVED				0x01			/* Y_SELF_TEST REG[7:0]*/
#define BMG250_REG__ERR_REG					0x02	/* DATASHEET(p. 27)/*/
#define BMG250_REG_PMU_STATUS				0x03	/* DATASHEET(p. 28)/*/
/*#define BMG250_REG RESERVED				0x04   */
/*#define BMG250_REG_							0x05	/* XG_OFFSET TEMP COMPENSATION REG[7:0]*/
/*#define BMG250_REG_							0x06*/
/*#define BMG250_REG RESERVED				0x04   */
#define BMG250_REG_GYR_X_0_7				0x12	/* X OUT [7:0] DATASHEET(p. 29)*/
#define BMG250_REG_GYR_X_8_15				0x13	/* X OUT [8:15] DATASHEET(p. 29)*/
#define BMG250_REG_GYR_Y_0_7				0x14	/* Y OUT [7:0] DATASHEET(p. 29)*/
#define BMG250_REG_GYR_Y_8_15				0x15	/* Y OUT [8:15] DATASHEET(p. 29)*/
#define BMG250_REG_GYR_Z_0_7				0x16	/* Z OUT [7:0] DATASHEET(p. 29)*/
#define BMG250_REG_GYR_Z_8_15				0x17	/* Z OUT [8:15] DATASHEET(p. 29)*/
#define BMG250_REG_SENSOR_TIME_0			0x18	/* SENSOR_TIME [0:7] DATASHEET(p. 30)*/
#define BMG250_REG_SENSOR_TIME_1			0x19	/* SENSOR_TIME [8:15] DATASHEET(p. 30)*/
#define BMG250_REG_SENSOR_TIME_2			0x1A	/* SENSOR_TIME	[16:23] DATASHEET(p. 30)*/

/* not  #define BMG250_REG_					0x09	/* INTERNEL CONTROL REGS*/
/* not  #define BMG250_REG_					0x0A	/* ZG_OFFSET TEMP COMPENSATION REG[9:8]*/
#define BMG250_REG_STATUS					0x1B	/*  DATASHEET(p. 31)*/
/*
 * **
 */
#define BMG250_REG_INT_STATUS_1				0x1D 	/* DATASHEET(p. 31)**/
#define BMG250_REG_TEMPERATURE_0			0x20	/* TEMPERATURE DATA [7:0] DATASHEET(p. 32)*/
#define BMG250_REG_TEMPERATURE_1			0x21	/* TEMPERATURE DATA [15:8] DATASHEET(p. 32)*/
#define BMG250_REG_FIFO_LENGTH_0			0x22	/* FIFO COUNTER [7:0] DATASHEET(p. 33)*/
#define BMG250_REG_FIFO_LENGTH_1			0x23	/* FIFO COUNTER [10:8] DATASHEET(p. 33)*/

#define BMG250_REG_FIFO_DATA				0x24	/* FIFO dataa [7:0] DATASHEET(p. 34)*/
#define BMG250_REG_GYR_CONF					0x42	/* GYRO CONFIG DATA RATE AND LPF DATASHEET(p. 35) */
#define BMG250_REG_GYR_RANGE				0x43	/* GYRO CONFIG DATA RANGE DATASHEET(p. 36)*/
#define BMG250_REG_FIFO_DOWNS				0x45	/* Z USER DEFINED OFFSET [7:0]*/

#define BMG250_REG_FIFO_CONFIG_0			0x46	/* SAMPLE RATE DIV reg DATASHEET(p.42)*/
																/* SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) Where INTERNAL_SAMPLE_RATE = 1kHz */
#define BMG250_REG_FIFO_CONFIG_1			0x47	/* SAMPLE RATE DIV reg DATASHEET(p.42)*/

#define BMG250_REG_INT_EN_1					0x51	/* CONFIG reg DATASHEET(p.43) **fifo enable **external snc **DLPF filte see table */
#define BMG250_REG_INT_OUT_CTRL				0x53	/* CONFIG reg DATASHEET(p.43) **fifo enable **external snc **DLPF filte see table */
#define BMG250_REG_INT_LATCH				0x54	/* CONFIG reg DATASHEET(p.43) **fifo enable **external snc **DLPF filte see table */
#define BMG250_REG_INT_MAP_1				0x56	/* CONFIG reg DATASHEET(p.43) **fifo enable **external snc **DLPF filte see table */

#define BMG250_REG_CONF						0x6A	/* GYRO_CONFIG reg DATASHEET(p.44) **full scale select **FBCHOISE for filter */

/*
 * **
 */
#define BMG250_REG_IF_CONF					0x6B	/* FIFO enable reg for x,y,z and temp DATASHEET(p.45) */
#define BMG250_REG_PMU_TRIGGER				0x6C	/*  FSYNC INTERRUPT STATUS DATASHEET(p.46)*/
#define BMG250_REG_SELF_TEST				0x6D	/*  INT PIN / BYPASS ENABLE CONFIGURATION  DATASHEET(p.46)*/
#define BMG250_REG_NV_CONF					0x70	/* INTERRUPT ENABLE DATASHEET(p.47)*/
#define BMG250_REG_OFFSET_3					0x74	/* INTERRUPT STATUS DATASHEET(p.47)*/
#define BMG250_REG_OFFSET_4					0x75	/* INTERRUPT STATUS DATASHEET(p.47)*/
#define BMG250_REG_OFFSET_5					0x76	/* INTERRUPT STATUS DATASHEET(p.47)*/
#define BMG250_REG_OFFSET_6					0x77	/* INTERRUPT STATUS DATASHEET(p.47)*/


#define BMG250_REG_CMD						0x7E	/* TEMP OUT Reg [15:8] DATASHEET(p.48)*/
 /*  DEBUG DEFINES */
#define BMG250_START_CODE  					0xAF

union BMG250_GYR_RANGE{
	struct{
		uint8_t Gyr_range0 :1;
		uint8_t Gyr_range1 :1;
		uint8_t Gyr_range2 :1;
		uint8_t reserved :5;

			}regbit;
		uint8_t Gyr_range :8;
};
typedef struct __attribute__ ((packed)){
  uint8_t headerStartCode;
  uint8_t chipID;
   float odr; //sampling frequency in Hz
   float range;
   float lpf;
   float hpf;
}BMG250_header;

typedef struct __attribute__ ((packed)){

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
	uint8_t dataStartCode;
	uint8_t IsReadingBmgFlag;
	/* angular rate  (X, Y, Z) in Gauss */

	uint8_t BmgGyro[6];

	/* Temperature data in deg */
	uint8_t BmgTemp[2];

} BMG;


typedef struct __attribute__ ((packed)){
 	  uint32_t m_seconds;
 	  uint16_t u_seconds;
}BMG_TS;

/*
 * INITIALISATION
 */
//HAL_StatusTypeDef BMG250_device_reset(BMG *dev, I2C_HandleTypeDef *i2cHandle  );

uint8_t BMG250_read_pmu_status(BMG *dev, I2C_HandleTypeDef *i2cHandle  );
uint8_t BMG250_read_err_reg(BMG *dev, I2C_HandleTypeDef *i2cHandle  );
uint8_t BMG250_self_test( BMG *dev, I2C_HandleTypeDef *i2cHandle );



uint8_t BMG250CheckId(BMG *dev, I2C_HandleTypeDef *i2cHandle ,BMG250_header *header );
uint8_t BMG250_SoftResetAndInit( BMG *dev, I2C_HandleTypeDef *i2cHandle ,BMG250_header *header) ;
//uint8_t BMG250_power_mode( BMG *dev, I2C_HandleTypeDef *i2cHandle );


//uint8_t BMG250_Initialise( BMG *dev, I2C_HandleTypeDef *i2cHandle,BMG250_header *header  );

HAL_StatusTypeDef BMG250_ReadTemperature( BMG *dev );
HAL_StatusTypeDef BMG250_ReadGyro( BMG *dev );


/*
	 * I2C_ FUNCTIONS
	 */
	HAL_StatusTypeDef BMG250_ReadRegister(  BMG *dev, uint8_t reg, uint8_t *data );
	HAL_StatusTypeDef BMG250_ReadRegisters( BMG *dev, uint8_t reg, uint8_t *data, uint8_t length );

	HAL_StatusTypeDef BMG250_WriteRegister( BMG *dev, uint8_t reg, uint8_t *data );


/*
	 *
	 * I2C_DMA
*/
	HAL_StatusTypeDef BMG250_ReadRegisters_DMA( BMG *dev, uint8_t reg, uint8_t *data, uint8_t length );
	HAL_StatusTypeDef BMG250_ReadGyro_DMA(BMG *dev);
	void 	BMG250_ReadGyroscopeDMA_Complete(BMG *dev);

#endif /* INC_BMG250_H_ */
