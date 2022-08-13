/*
 *
 * ADXL355.h
 *
 * Author:	Gaming
 * Created: 11 November 2021
 *
 */


#ifndef INC_ADXL355_H_
#define INC_ADXL355_H_


#include "stm32l4xx_hal.h" /* inc Hal_lib */

/*
 * DEFINES
 */
#define ADXL355_I2C_ADDR	(0x1D << 1) /*I2C 7 BITS ADRESS : ASEL = 0 -> 0x1D OR ASEL = 1 -> 0x53 DATASHEET (p. 26) */

#define ADXL355_DEVICE_ID	0xAD   /*DEVICE ID  -READ_ONLY DEFINED BY FACTORY :USE TO CHECK THE SENSOR -*/
#define ADXL355_MEMS_ID		0x1D    /* DEVICE MEMS ID  -READ_ONLY DEFINED BY FACTORY"""*/
#define ADXL355_PART_ID		0xED 	/* DEVICE PART ID  REG ADRESS-READ_ONLY DEFINED BY FACTORY/

/*
 * REGISTERS MAP DATASHEET(p. 31)  *PS: ALL REGISTERS ARE 8 BITS
 */
#define ADXL355_REG_DEVID_AD		0x00 	/* DEVICE  ID  REG ADRESS*/
#define ADXL355_REG_DEVID_MST		0x01	/* DEVICE  ID  REG ADRESS*/
#define ADXL355_REG_PARTID			0x02	/* DEVICE  ID  REG ADRESS*/
#define ADXL355_REG_REVID			0x03	/* DEVICE  ID  REG ADRESS*/
#define ADXL355_REG_STATUS			0x04	/* STATUS REG ADRESS DATASHEET(p. 32) */
#define ADXL355_REG_FIFO_ENTRIES	0x05	/*  DATASHEET(p. 33) */
#define ADXL355_REG_TEMP2			0x06
#define ADXL355_REG_TEMP1			0x07
#define ADXL355_REG_XDATA3			0x08
#define ADXL355_REG_XDATA2			0x09
#define ADXL355_REG_XDATA1			0x0A
#define ADXL355_REG_YDATA3			0x0B	/*  DATASHEET(p. 34) */
#define ADXL355_REG_YDATA2			0x0C
#define ADXL355_REG_YDATA1			0x0D
#define ADXL355_REG_ZDATA3			0x0E
#define ADXL355_REG_ZDATA2			0x0F
#define ADXL355_REG_ZDATA1			0x10
#define ADXL355_REG_FIFO_DATA		0x11	/*  DATASHEET(p. 35) */
#define ADXL355_REG_OFFSET_X_H		0x1E
#define ADXL355_REG_OFFSET_X_L		0x1F
#define ADXL355_REG_OFFSET_Y_H		0x20
#define ADXL355_REG_OFFSET_Y_L		0x21
#define ADXL355_REG_OFFSET_Z_H		0x22	/*  DATASHEET(p. 36) */
#define ADXL355_REG_OFFSET_Z_L		0x23
#define ADXL355_REG_ACT_EN			0x24
#define ADXL355_REG_ACT_THRESH_H	0x25
#define ADXL355_REG_ACT_THRESH_L	0x26
#define ADXL355_REG_ACT_COUNT		0x27
#define ADXL355_REG_FILTER			0x28 	/*  DATASHEET(p. 37) */
#define ADXL355_REG_FIFO_SAMPLES	0x29
#define ADXL355_REG_INT_MAP			0x2A
#define ADXL355_REG_SYNC			0x2B	/*  DATASHEET(p. 38) */
#define ADXL355_REG_RANGE			0x2C
#define ADXL355_REG_POWER_CTL		0x2D
#define ADXL355_REG_SELF_TEST		0x2E	/*  DATASHEET(p. 39) */
#define ADXL355_REG_ESET			0x2F
/*
 * SENSOR STRUCT
 */
typedef struct __attribute__ ((packed)) {
  uint8_t headerStartCode;// = 0xA5;
  uint8_t devid_ad ;
  uint8_t devid_mst;
  uint8_t partid;
  uint8_t revid;
 //  timestamp ts; declare it as a global variable or include main.h ?
  float odr; //sampling frequency in Hz
  float range;
  float lpf;
  float hpf;
}ADXL355_header;

typedef struct __attribute__ ((packed)) {

	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;
	  uint8_t dataStartCode; // = 0xAA;
	  uint8_t IsReadingAdxlFlag;


	/* Acceleration register unsiged raw  data  from Reg XDATA3 &ADD 0x08 to reg ZDATA3 &ADD 0x10 */
	uint8_t AdxlAcc[9];

	/* Temperature data in deg */
	uint8_t AdxlTemp[2];


} ADXL355;

typedef struct __attribute__ ((packed)){
 	  uint32_t m_seconds;
 	  uint16_t u_seconds;
}ADXL355_TS;


/*
 * INITIALISATION
 */
uint8_t ADXL355_Initialise( ADXL355 *dev, I2C_HandleTypeDef *i2cHandle,ADXL355_header *header );

/*
 * Headers
 */

//7void ADXL_getHeader(ADXL_SPI adxl_spi, ADXL_Header *header)
/*
 * DATA ACQUISITON
 */
HAL_StatusTypeDef ADXL355_ReadTemperature( ADXL355 *dev );
HAL_StatusTypeDef ADXL355_ReadAccelerations( ADXL355 *dev );

/*
 * I2C_ FUNCTIONS
 */
HAL_StatusTypeDef ADXL355_ReadRegister(  ADXL355 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef ADXL355_ReadRegisters( ADXL355 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef ADXL355_WriteRegister( ADXL355 *dev, uint8_t reg, uint8_t *data );


#endif
