#include "mpu6050.h"

int16_t lsb_sensitivity[4] = {16384, 8192, 4096, 2048};

void mpu6050_init(struct mpu6050 *mpu6050_ptr, i2c_master_dev_handle_t i2c_dev_handle)
{
		mpu6050_ptr->i2c_dev_handle = i2c_dev_handle;
		mpu6050_ptr->accel_fs_sel = AFS_SEL_DEFAULT;
		mpu6050_ptr->accel_lsb_sens = lsb_sensitivity[mpu6050_ptr->accel_fs_sel];
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
	int8_t accel_xout_h = mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_XOUT_H);
	int8_t accel_xout_l = mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_XOUT_L);
	int8_t accel_yout_h = mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_YOUT_H);
	int8_t accel_yout_l = mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_YOUT_L);
	int8_t accel_zout_h = mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_ZOUT_H);
	int8_t accel_zout_l = mpu6050_readRegister(mpu6050_ptr, MPU6050_REG_ACCEL_ZOUT_L);

	mpu6050_ptr->accel_x_out = ((accel_xout_h << 8) | accel_xout_l) / (float) lsb_sensitivity[mpu6050_ptr->accel_fs_sel];
	mpu6050_ptr->accel_y_out = ((accel_yout_h << 8) | accel_yout_l) / (float) lsb_sensitivity[mpu6050_ptr->accel_fs_sel];
	mpu6050_ptr->accel_z_out = ((accel_zout_h << 8) | accel_zout_l) / (float) lsb_sensitivity[mpu6050_ptr->accel_fs_sel];
}
