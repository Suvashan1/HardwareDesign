/*
 * bmp180.c
 *
 *  Created on: Jan 7, 2021
 *      Author: Suvashan
 */

#include "bmp180.h"
#include "math.h"

//Create variables for the oversampling setting, calibration co-efficients and I2C handle
BMP180_OSS oss;
BMP180_calib calib;
I2C_HandleTypeDef *i2c;


const uint8_t  temp_ctrl = 0x2E;
const uint8_t temp_delay = 5;
const uint8_t press_ctrl[4] = {0x34, 0x74, 0xB4, 0xF4};
const uint8_t press_delay[4] = {5, 8, 14, 26};

const uint8_t bmp180_reg_addr_MSB[11] = {0xaa, 0xac, 0xae, 0xb0, 0xb2, 0xb4, 0xb6, 0xb8, 0xba, 0xbc, 0xbe};
const uint8_t bmp180_reg_addr_LSB[11] = {0xab, 0xad, 0xaf, 0xb1, 0xb3, 0xb5, 0xb7, 0xb9, 0xbb, 0xbd, 0xbf};



static void writeData(uint8_t reg, uint8_t data){

	uint8_t arr[2] = {reg, data};

	HAL_I2C_Master_Transmit(i2c, (BMP180_I2C_ADDR << 1), arr, 2, 1000);
}

static uint8_t readData(uint8_t reg){

		HAL_I2C_Master_Transmit(i2c, (BMP180_I2C_ADDR << 1) , &reg, 1, 1000);
		uint8_t data = 0;
		HAL_I2C_Master_Receive(i2c, (BMP180_I2C_ADDR << 1) , &data, 1, 1000);
		return data;
}

/**
 * @brief Retrieves calibration data
 *
 */
void _get_calib_data(void){

	calib.AC1 = (readData(bmp180_reg_addr_MSB[AC1_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[AC1_INDEX]);
	calib.AC2 = (readData(bmp180_reg_addr_MSB[AC2_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[AC2_INDEX]);
	calib.AC3 = (readData(bmp180_reg_addr_MSB[AC3_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[AC3_INDEX]);
	calib.AC4 = (readData(bmp180_reg_addr_MSB[AC4_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[AC4_INDEX]);
	calib.AC5 = (readData(bmp180_reg_addr_MSB[AC5_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[AC5_INDEX]);
	calib.AC6 = (readData(bmp180_reg_addr_MSB[AC6_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[AC6_INDEX]);
	calib.B1 = (readData(bmp180_reg_addr_MSB[B1_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[B1_INDEX]);
	calib.B2 = (readData(bmp180_reg_addr_MSB[B2_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[B2_INDEX]);
	calib.MB = (readData(bmp180_reg_addr_MSB[MB_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[MB_INDEX]);
	calib.MC = (readData(bmp180_reg_addr_MSB[MC_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[MC_INDEX]);
	calib.MD = (readData(bmp180_reg_addr_MSB[MD_INDEX]) << 8) | readData(bmp180_reg_addr_LSB[MD_INDEX]);
}

/**
 * @brief Initialises the I2C Handle for the respective I2Cx peripheral, and the oversampling setting
 * @param bmp180_i2c pointer to the I2C Handle
 * @param bmp180_oss oversampling setting (options: ultraLP, standard, highRes, ultraHigh)
 */
void bmp180_oss_i2c_init(I2C_HandleTypeDef *bmp180_i2c, BMP180_OSS bmp180_oss){

	i2c = bmp180_i2c;
	oss = bmp180_oss;
}

/**
 * @brief calculates the temperature(in degrees celsius) based on the logic and mathematics given in BMP180 datasheet
 * @return temperature value
 */

int32_t _get_Temp(void){

	writeData(BMP180_CTRL_REG, temp_ctrl);
	HAL_Delay(temp_delay);
	long uncompT = (readData(BMP180_MSB) << 8) | (readData(BMP180_LSB));
	long X1 = (uncompT - calib.AC6) * calib.AC5/(1<<15);
	long X2 = (calib.MC * (1 << 11))/(X1 + calib.MD);
	long B5 = X1 + X2;
	return (B5+8)/ (1 << 4);
}

/**
 * @brief calculates the pressure(in Pa) based on the logic and mathematics given in BMP180 datasheet
 * @return pressure value
 */
int32_t _get_Pressure(void){
	writeData(BMP180_CTRL_REG, temp_ctrl);
	HAL_Delay(temp_delay);
	int32_t tempUT = bmp180_getUT();
	writeData(BMP180_CTRL_REG, press_ctrl[oss]);
	HAL_Delay(press_delay[oss]);
	int32_t tempUP = bmp180_getUP();
	long X1 = (tempUT - calib.AC6) * calib.AC5/(1<<15);
	long X2 = (calib.MC * (1 << 11))/(X1 + calib.MD);
	long B5 = X1 + X2;
	long B6 = B5 - 4000;
	X1 = (calib.B2 * (B6 * B6 / (1 << 12))) / (1 << 11);
	X2 = calib.AC2 * B6/ (1 << 11);
	long X3 = X1 + X2;
	long B3 = (((calib.AC1 * 4 + X3) << oss)  + 2) / (1 << 2);
	X1 = calib.AC3 * B6 / (1 << 15);
	X2 = (calib.B1 * (B6 * B6 / (1 << 12))) / (1 << 16);
	X3 = ((X1 + X2) + 2) / 4;
	uint32_t  B4 = calib.AC4 * (uint32_t)(X3 + 32768) / (1 << 15);
	uint32_t B7 = ((uint32_t)tempUP - B3) * (50000 >> oss);
	int32_t p;
	if (B7 < 0x80000000){
		p = (B7 * 2) / B4;
	}
	else{
		p = (B7/B4) * 2;
	}

	X1 = (p/(1 << 8)) * (p/(1 << 8));
	X1 = (X1 * 3038) / (1 << 16);
	X2 = (-7357 * p)/ (1 << 16);
	p = p + (X1 + X2 + 3791)/ (1 << 4);
	return p;
}

static int32_t bmp180_getUT(void){
	return (readData(BMP180_MSB) << 8) | readData(BMP180_LSB);
}

static int32_t bmp180_getUP(void){

	return ((readData(BMP180_MSB) << 16) | (readData(BMP180_LSB) << 8) | readData(BMP180_XLSB)) >> (8 - oss);

}








