#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#include "mpu6050_registers.h"

#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR       	0x68        /*!< Address of the sensor */

#define PEDOMETER_ACCEL_THRESHOLD	6	

#define AFS_SEL_2G					0
#define AFS_SEL_4G					1
#define AFS_SEL_8G					2
#define AFS_SEL_16G					3
#define AFS_SEL_DEFAULT				AFS_SEL_8G

struct mpu6050
{
    i2c_master_dev_handle_t i2c_dev_handle;
	uint8_t accel_fs_sel;
	float accel_lsb_sens;
	float accel_x_out;
	float accel_y_out;
	float accel_z_out;
};

void mpu6050_init(struct mpu6050 *mpu6050_ptr, i2c_master_dev_handle_t i2c_dev_handle);
uint8_t mpu6050_readRegister(struct mpu6050 *mpu6050_ptr, uint8_t address);
void mpu6050_writeRegister(struct mpu6050 *mpu6050_ptr, uint8_t address, uint8_t data);
void mpu6050_burstRead(struct mpu6050 *mpu6050_ptr, uint8_t baseAddress, uint8_t *buffer, uint8_t length);

void mpu6050_getAccel(struct mpu6050 *mpu6050_ptr);

#endif //__MPU6050_H__
