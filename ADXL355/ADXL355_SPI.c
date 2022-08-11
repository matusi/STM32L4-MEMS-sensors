/*
 * ADXL355.c
 *
 *  Created on: Nov 10, 2021
 *      Author: Gaming
 */

#include "ADXL355_SPI.h"
#include <math.h>

void ADXL_getHeader(ADXL_SPI adxl_spi, ADXL_Header *header) {
	uint8_t temp = 0;
	ADXL_ReadRegister(adxl_spi, ADXL355_REG_DEVID_AD, &(header->devid_ad), 4);
	//header->ts.m_seconds = t_s / 1000;
	//header->ts.u_seconds = t_s % 1000;
	ADXL_ReadRegister(adxl_spi, ADXL355_REG_FILTER, &temp, 1);
	header->odr = 4000.0 / pow(2, (temp & 0x0F));
	header->lpf = 1000.0 / pow(2, (temp & 0x0F));

	ADXL_ReadRegister(adxl_spi, ADXL355_REG_RANGE, &temp, 1);
	switch ((0x03 & temp)) {
		case 1: header->range = 2;
				break;
		case 2: header->range = 4;
				break;
		case 3: header->range = 8;
				break;
		default: header->range = 0;
	}
}

void ADXL_set_ODR_RANGE(ADXL_SPI adxl_spi,uint8_t odr, uint8_t range){

}



void ADXL_ReadRegister(ADXL_SPI adxl_spi, uint8_t regAddr, uint8_t *data, uint8_t length){

	HAL_GPIO_WritePin(adxl_spi.CS_GPIO, adxl_spi.CS_PIN, GPIO_PIN_RESET);
	uint8_t addr = (regAddr<<1) | 0x01;
	HAL_SPI_Transmit(adxl_spi.adxl_spi, &addr , 1 , 1);
	HAL_SPI_Receive(adxl_spi. adxl_spi, data,length , 1);
	HAL_GPIO_WritePin(adxl_spi.CS_GPIO, adxl_spi.CS_PIN, GPIO_PIN_SET);

}


void ADXL_WriteRegister(ADXL_SPI adxl_spi, uint8_t regAddr, uint8_t data){
	HAL_GPIO_WritePin(adxl_spi.CS_GPIO, adxl_spi.CS_PIN, GPIO_PIN_RESET);
	uint8_t addr = (regAddr<<1) | 0x00;
	HAL_SPI_Transmit(adxl_spi.adxl_spi, &addr , 1 , 1);
	HAL_SPI_Transmit(adxl_spi.adxl_spi, &data, 1 , 1);
	HAL_GPIO_WritePin(adxl_spi.CS_GPIO, adxl_spi.CS_PIN, GPIO_PIN_SET);
}
