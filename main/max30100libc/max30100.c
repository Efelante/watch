/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h> 
#include <unistd.h> 

#include "circular_buffer.h"
#include "max30100.h"

void max30100_init(struct max30100 *max30100_ptr, 
		i2c_master_dev_handle_t i2c_dev_handle,
		cbuf_handle_t cbuf)
{
		max30100_ptr->i2c_dev_handle = i2c_dev_handle;
		max30100_ptr->readoutsBuffer = cbuf;
}

bool max30100_begin(struct max30100 *max30100_ptr)
{
    if (max30100_getPartId(max30100_ptr) != EXPECTED_PART_ID) {
        return false;
    }

    max30100_setMode(max30100_ptr, DEFAULT_MODE);
    max30100_setLedsPulseWidth(max30100_ptr, DEFAULT_PULSE_WIDTH);
    max30100_setSamplingRate(max30100_ptr, DEFAULT_SAMPLING_RATE);
    max30100_setLedsCurrent(max30100_ptr, DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    max30100_setHighresModeEnabled(max30100_ptr, true);
	max30100_writeRegister(max30100_ptr, MAX30100_REG_FIFO_READ_POINTER, 0x00);
	max30100_writeRegister(max30100_ptr, MAX30100_REG_FIFO_WRITE_POINTER, 0x00);
	max30100_writeRegister(max30100_ptr, MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0x00);

    return true;
}

uint8_t max30100_readRegister(struct max30100 *max30100_ptr, uint8_t address)
{

	uint8_t data;
	i2c_master_transmit_receive(max30100_ptr->i2c_dev_handle, &address, 1,
			&data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
	//TODO: add dev_handle to the class

	return data;
}

void max30100_writeRegister(struct max30100 *max30100_ptr, uint8_t address, uint8_t data)
{
    uint8_t write_buf[2] = {address, data};
	i2c_master_transmit(max30100_ptr->i2c_dev_handle, write_buf,
			sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void max30100_burstRead(struct max30100 *max30100_ptr, uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
	i2c_master_transmit_receive(max30100_ptr->i2c_dev_handle, &baseAddress,
			1, buffer, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void max30100_readFifoData(struct max30100 *max30100_ptr)
{
	//uint8_t max30100_write_ptr = max30100_readRegister(max30100_ptr, MAX30100_REG_FIFO_WRITE_POINTER);
	//uint8_t max30100_read_ptr  = max30100_readRegister(max30100_ptr, MAX30100_REG_FIFO_READ_POINTER);
	//uint8_t toRead = (max30100_write_ptr - max30100_read_ptr) & (MAX30100_FIFO_DEPTH - 1);
	//printf("WR_PTR: %i\n", max30100_write_ptr);
	//printf("RD_PTR: %i\n", max30100_read_ptr);
	//printf("to read: %i\n", toRead);
	//uint8_t raw_data[4] = {0};
	//max30100_burstRead(max30100_ptr, MAX30100_REG_FIFO_DATA, raw_data, 4);
	//uint16_t raw_hr = (raw_data[0] << 8) | raw_data[1];
	//uint16_t raw_spo2 = (raw_data[2] << 8) | raw_data[3];
	//printf("raw HR = %X raw SPO2 = %X\n", raw_hr, raw_spo2);

    uint8_t buffer[MAX30100_FIFO_DEPTH*4];
    uint8_t toRead;
	uint8_t write_ptr = max30100_readRegister(max30100_ptr, MAX30100_REG_FIFO_WRITE_POINTER);
	uint8_t read_ptr  = max30100_readRegister(max30100_ptr, MAX30100_REG_FIFO_READ_POINTER);
	uint8_t ovf_cnt   = max30100_readRegister(max30100_ptr, MAX30100_REG_FIFO_OVERFLOW_COUNTER);

	if (ovf_cnt > 0) {
		toRead = 16;
	} else {
		if (read_ptr > write_ptr) {
			toRead = 16 - read_ptr + write_ptr;
		} else {
			toRead = write_ptr - read_ptr;
		}
	}
	//printf("WR_PTR: %i, RD_PTR: %i, to read: %i\n", write_ptr, read_ptr, toRead);

    if (toRead) {
		//printf("to read: %i\n", toRead);
        max30100_burstRead(max30100_ptr, MAX30100_REG_FIFO_DATA, buffer, 4 * toRead);

        for (uint8_t i=0 ; i < toRead ; ++i) {
			struct max30100_data data = {
                    (uint16_t)((buffer[i*4] << 8) | buffer[i*4 + 1]),
                    (uint16_t)((buffer[i*4 + 2] << 8) | buffer[i*4 + 3])
					};
            // Warning: the values are always left-aligned
            circular_buf_put(max30100_ptr->readoutsBuffer, data);
        }
    }
}

uint8_t max30100_getPartId(struct max30100 *max30100_ptr)
{
    return max30100_readRegister(max30100_ptr, 0xff);
}

void max30100_setMode(struct max30100 *max30100_ptr, Mode mode)
{
    max30100_writeRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION, mode);
}

void max30100_setLedsPulseWidth(struct max30100 *max30100_ptr, LEDPulseWidth ledPulseWidth)
{
    uint8_t previous = max30100_readRegister(max30100_ptr, MAX30100_REG_SPO2_CONFIGURATION);
    max30100_writeRegister(max30100_ptr, MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void max30100_setSamplingRate(struct max30100 *max30100_ptr, SamplingRate samplingRate)
{
    uint8_t previous = max30100_readRegister(max30100_ptr, MAX30100_REG_SPO2_CONFIGURATION);
    max30100_writeRegister(max30100_ptr, MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void max30100_setLedsCurrent(struct max30100 *max30100_ptr, LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    max30100_writeRegister(max30100_ptr, MAX30100_REG_LED_CONFIGURATION, redLedCurrent << 4 | irLedCurrent);
}

void max30100_setHighresModeEnabled(struct max30100 *max30100_ptr, bool enabled)
{
    uint8_t previous = max30100_readRegister(max30100_ptr, MAX30100_REG_SPO2_CONFIGURATION);
    if (enabled) {
        max30100_writeRegister(max30100_ptr, MAX30100_REG_SPO2_CONFIGURATION, previous | MAX30100_SPC_SPO2_HI_RES_EN);
    } else {
        max30100_writeRegister(max30100_ptr, MAX30100_REG_SPO2_CONFIGURATION, previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
    }
}

void max30100_update(struct max30100 *max30100_ptr)
{
    max30100_readFifoData(max30100_ptr);
}

bool max30100_getRawValues(struct max30100 *max30100_ptr, uint16_t *ir, uint16_t *red)
{
    if (!circular_buf_empty(max30100_ptr->readoutsBuffer)) {
        struct max30100_data readout;
		circular_buf_get(max30100_ptr->readoutsBuffer, &readout);

        *ir = readout.ir;
        *red = readout.red;

        return true;
    } else {
        return false;
    }
}

void max30100_resetFifo(struct max30100 *max30100_ptr)
{
    max30100_writeRegister(max30100_ptr, MAX30100_REG_FIFO_WRITE_POINTER, 0);
    max30100_writeRegister(max30100_ptr, MAX30100_REG_FIFO_READ_POINTER, 0);
    max30100_writeRegister(max30100_ptr, MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0);
}

void max30100_startTemperatureSampling(struct max30100 *max30100_ptr)
{
    uint8_t modeConfig = max30100_readRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_TEMP_EN;

    max30100_writeRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

bool max30100_isTemperatureReady(struct max30100 *max30100_ptr)
{
    return !(max30100_readRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION) & MAX30100_MC_TEMP_EN);
}

float max30100_retrieveTemperature(struct max30100 *max30100_ptr)
{
    int8_t tempInteger = max30100_readRegister(max30100_ptr, MAX30100_REG_TEMPERATURE_DATA_INT);
    float tempFrac = max30100_readRegister(max30100_ptr, MAX30100_REG_TEMPERATURE_DATA_FRAC);

    return tempFrac * 0.0625 + tempInteger;
}

void max30100_shutdown(struct max30100 *max30100_ptr)
{
    uint8_t modeConfig = max30100_readRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_SHDN;

    max30100_writeRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

void max30100_resume(struct max30100 *max30100_ptr)
{
    uint8_t modeConfig = max30100_readRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION);
    modeConfig &= ~MAX30100_MC_SHDN;

    max30100_writeRegister(max30100_ptr, MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}
