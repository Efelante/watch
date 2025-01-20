#include <unistd.h>

#include "mpu6050.h"

int16_t lsb_sensitivity[4] = {16384, 8192, 4096, 2048};

void mpu6050_init(struct mpu6050 *mpu6050_ptr, i2c_master_dev_handle_t i2c_dev_handle)
{
		mpu6050_ptr->i2c_dev_handle = i2c_dev_handle;
		mpu6050_ptr->accel_fs_sel = AFS_SEL_DEFAULT;
		mpu6050_ptr->accel_lsb_sens = lsb_sensitivity[mpu6050_ptr->accel_fs_sel];

		// mpu6050 reset
		mpu6050_writeRegister(mpu6050_ptr, MPU6050_REG_PWR_MGMT_1, 0x80);	// reset device
		usleep(100 * 1000);					
		usleep(100 * 1000);					
		mpu6050_writeRegister(mpu6050_ptr, MPU6050_REG_SIGNAL_PATH_RESET, 0x07);	// reset tmp, gyro, accel
		usleep(100 * 1000);					
		usleep(100 * 1000);					

		// mpu6050 set sample rate divisor
		mpu6050_writeRegister(mpu6050_ptr, MPU6050_REG_SMPLRT_DIV, 0x00);

		// Set filter bandwidth to 260 Hz
		mpu6050_writeRegister(mpu6050_ptr, MPU6050_REG_CONFIG, 0x00);

		// Set gyro range

		// Set accel range (2g - default)
		// Replace with 8g

		// set clock config to PLL with Gyro X reference
		mpu6050_writeRegister(mpu6050_ptr, MPU6050_REG_PWR_MGMT_1, 0x01);
		usleep(100 * 1000);					
}

uint8_t mpu6050_readRegister(struct mpu6050 *mpu6050_ptr, uint8_t address)
{

	uint8_t data;

	i2c_master_transmit_receive(mpu6050_ptr->i2c_dev_handle, &address, 1,
			&data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

	return data;
}

void mpu6050_writeRegister(struct mpu6050 *mpu6050_ptr, uint8_t address, uint8_t data)
{
    uint8_t write_buf[2] = {address, data};
	i2c_master_transmit(mpu6050_ptr->i2c_dev_handle, write_buf,
			sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void mpu6050_burstRead(struct mpu6050 *mpu6050_ptr, uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
	i2c_master_transmit_receive(mpu6050_ptr->i2c_dev_handle, &baseAddress,
			1, buffer, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void mpu6050_getAccel(struct mpu6050 *mpu6050_ptr)
{
	uint8_t data[6] = {0};
	uint8_t *accel_xout_h = &data[0];//mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_XOUT_H);
	uint8_t *accel_xout_l = &data[1];//mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_XOUT_L);
	uint8_t *accel_yout_h = &data[2];//mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_YOUT_H);
	uint8_t *accel_yout_l = &data[3];//mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_YOUT_L);
	uint8_t *accel_zout_h = &data[4];//mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_ZOUT_H);
	uint8_t *accel_zout_l = &data[5];//mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_ZOUT_L);

	mpu6050_burstRead(mpu6050_ptr, MPU6050_REG_ACCEL_XOUT_H, data, 6);

	int16_t rawAccelX = (*accel_xout_h << 8) | *accel_xout_l;
	int16_t rawAccelY = (*accel_yout_h << 8) | *accel_yout_l;
	int16_t rawAccelZ = (*accel_zout_h << 8) | *accel_zout_l;

	mpu6050_ptr->accel_x_out = ((float) rawAccelX) / mpu6050_ptr->accel_lsb_sens;
	mpu6050_ptr->accel_y_out = ((float) rawAccelY) / mpu6050_ptr->accel_lsb_sens;
	mpu6050_ptr->accel_z_out = ((float) rawAccelZ) / mpu6050_ptr->accel_lsb_sens;
}
