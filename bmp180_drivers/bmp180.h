/**
 * The following header file forms part of a device drivers for the BMP180 temperature and pressure sensor, to
 * be used with an STM32 microcontroller or development board. It uses the HAL library, written by STMicroelectronics.
 * The file below should be altered to include the header file of the microcontroller in use. (line 12 and 13)
 *
 */

#ifndef BMP180_H_
#define BMP180_H_

#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#define BMP180_I2C_ADDR				0x77
#define BMP180_CTRL_REG				0xf4
#define BMP180_ID_REG				0xd0
#define BMP180_ID					0x55
#define BMP180_XLSB					0xf8
#define BMP180_LSB					0xf7
#define BMP180_MSB					0xf6

/**
 * @brief oversampling setting options (explained on page 12 of the BMP180 datasheet)
 */
typedef enum BMP180_OSS{

	ultraLP,
	standard,
	highRes,
	ultraHigh,

}BMP180_OSS;

/**
 * @brief calibration co-efficients used in calculation of temperature and pressure
 */
typedef struct BMP180_calib{

	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
}BMP180_calib;

typedef enum calib_index{

	AC1_INDEX,
	AC2_INDEX,
	AC3_INDEX,
	AC4_INDEX,
	AC5_INDEX,
	AC6_INDEX,
	B1_INDEX,
	B2_INDEX,
	MB_INDEX,
	MC_INDEX,
	MD_INDEX,

}calib_index;


extern const uint8_t  temp_ctrl;
extern const uint8_t temp_delay;
extern const uint8_t press_ctrl[4];
extern const uint8_t press_delay[4];

extern const uint8_t bmp180_reg_addr_MSB[11];
extern const uint8_t bmp180_reg_addr_LSB[11];

int32_t _get_Temp(void);
void _get_calib_data(void);
int32_t _get_Pressure(void);
void bmp180_oss_i2c_init(I2C_HandleTypeDef *bmp180_i2c, BMP180_OSS bmp180_oss);


#endif /* BMP180_H_ */
