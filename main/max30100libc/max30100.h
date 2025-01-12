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

#ifndef __MAX30100_H__
#define __MAX30100_H__

#include <stdint.h>

#include "max30100_registers.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#define DEFAULT_MODE                MAX30100_MODE_SPO2_HR
#define DEFAULT_SAMPLING_RATE       MAX30100_SAMPRATE_100HZ
#define DEFAULT_PULSE_WIDTH         MAX30100_SPC_PW_1600US_16BITS
#define DEFAULT_RED_LED_CURRENT     MAX30100_LED_CURR_50MA
#define DEFAULT_IR_LED_CURRENT      MAX30100_LED_CURR_50MA
#define EXPECTED_PART_ID            0x11
#define RINGBUFFER_SIZE             16

#define I2C_BUS_SPEED               400000UL
#define I2C_MASTER_TIMEOUT_MS       1000

struct max30100_data {
    uint16_t ir;
    uint16_t red;
};

#include "circular_buffer.h"

struct max30100
{
    i2c_master_dev_handle_t i2c_dev_handle;
	cbuf_handle_t readoutsBuffer;
};

void 	max30100_init(struct max30100 *max30100_handle, i2c_master_dev_handle_t i2c_dev_handle, cbuf_handle_t cbuf);
void 	max30100_writeRegister(struct max30100 *max30100_handle, uint8_t address, uint8_t data);
uint8_t max30100_readRegister(struct max30100 *max30100_handle, uint8_t address);
void 	max30100_burstRead(struct max30100 *max30100_handle, uint8_t baseAddress, uint8_t *buffer, uint8_t length);
uint8_t max30100_getPartId(struct max30100 *max30100_ptr);
bool 	max30100_begin(struct max30100 *max30100_ptr);
void 	max30100_setMode(struct max30100 *max30100_ptr, Mode mode);
void 	max30100_setLedsPulseWidth(struct max30100 *max30100_ptr, LEDPulseWidth ledPulseWidth);
void 	max30100_setSamplingRate(struct max30100 *max30100_ptr, SamplingRate samplingRate);
void 	max30100_setLedsCurrent(struct max30100 *max30100_ptr, LEDCurrent irLedCurrent, LEDCurrent redLedCurrent);
void 	max30100_setHighresModeEnabled(struct max30100 *max30100_ptr, bool enabled);
void 	max30100_update(struct max30100 *max30100_ptr);
bool 	max30100_getRawValues(struct max30100 *max30100_ptr, uint16_t *ir, uint16_t *red);
void 	max30100_resetFifo(struct max30100 *max30100_ptr);
void 	max30100_startTemperatureSampling(struct max30100 *max30100_ptr);
bool 	max30100_isTemperatureReady(struct max30100 *max30100_ptr);
float 	max30100_retrieveTemperature(struct max30100 *max30100_ptr);
void 	max30100_shutdown(struct max30100 *max30100_ptr);
void 	max30100_resume(struct max30100 *max30100_ptr);
void 	max30100_readFifoData(struct max30100 *max30100_ptr);

#endif
