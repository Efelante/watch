#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#include "mpu6050_registers.h"

#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR       	0x68        /*!< Address of the sensor */

struct mpu6050
{
    i2c_master_dev_handle_t i2c_dev_handle;
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
