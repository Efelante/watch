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

//#include <Wire.h>

#include "MAX30100.h"

MAX30100::MAX30100()
{
}

MAX30100::MAX30100(i2c_master_dev_handle_t handle):
	dev_handle(handle)
{
}

bool MAX30100::begin()
{
    if (getPartId() != EXPECTED_PART_ID) {
        return false;
    }

    setMode(DEFAULT_MODE);
    setLedsPulseWidth(DEFAULT_PULSE_WIDTH);
    setSamplingRate(DEFAULT_SAMPLING_RATE);
    setLedsCurrent(DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
    setHighresModeEnabled(true);

    return true;
}

void MAX30100::setMode(Mode mode)
{
    writeRegister(MAX30100_REG_MODE_CONFIGURATION, mode);
}

void MAX30100::setLedsPulseWidth(LEDPulseWidth ledPulseWidth)
{
    uint8_t previous = readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void MAX30100::setSamplingRate(SamplingRate samplingRate)
{
    uint8_t previous = readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30100_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void MAX30100::setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    writeRegister(MAX30100_REG_LED_CONFIGURATION, redLedCurrent << 4 | irLedCurrent);
}

void MAX30100::setHighresModeEnabled(bool enabled)
{
    uint8_t previous = readRegister(MAX30100_REG_SPO2_CONFIGURATION);
    if (enabled) {
        writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous | MAX30100_SPC_SPO2_HI_RES_EN);
    } else {
        writeRegister(MAX30100_REG_SPO2_CONFIGURATION, previous & ~MAX30100_SPC_SPO2_HI_RES_EN);
    }
}

void MAX30100::update()
{
    readFifoData();
}

bool MAX30100::getRawValues(uint16_t *ir, uint16_t *red)
{
    if (!readoutsBuffer.isEmpty()) {
        SensorReadout readout = readoutsBuffer.pop();

        *ir = readout.ir;
        *red = readout.red;

        return true;
    } else {
        return false;
    }
}

void MAX30100::resetFifo()
{
    writeRegister(MAX30100_REG_FIFO_WRITE_POINTER, 0);
    writeRegister(MAX30100_REG_FIFO_READ_POINTER, 0);
    writeRegister(MAX30100_REG_FIFO_OVERFLOW_COUNTER, 0);
}

uint8_t MAX30100::readRegister(uint8_t address)
{

	uint8_t data;
    i2c_master_transmit_receive(dev_handle, &address, 1, &data, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
	//TODO: add dev_handle to the class

	return data;
}

void MAX30100::writeRegister(uint8_t address, uint8_t data)
{
    uint8_t write_buf[2] = {address, data};
    i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void MAX30100::burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
    i2c_master_transmit_receive(dev_handle, &baseAddress, 1, buffer, length, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void MAX30100::readFifoData()
{
    uint8_t buffer[MAX30100_FIFO_DEPTH*4];
    uint8_t toRead;

    toRead = (readRegister(MAX30100_REG_FIFO_WRITE_POINTER) - readRegister(MAX30100_REG_FIFO_READ_POINTER)) & (MAX30100_FIFO_DEPTH-1);

    if (toRead) {
        burstRead(MAX30100_REG_FIFO_DATA, buffer, 4 * toRead);

        for (uint8_t i=0 ; i < toRead ; ++i) {
            // Warning: the values are always left-aligned
            readoutsBuffer.push({
                    .ir=(uint16_t)((buffer[i*4] << 8) | buffer[i*4 + 1]),
                    .red=(uint16_t)((buffer[i*4 + 2] << 8) | buffer[i*4 + 3])});
        }
    }
}

void MAX30100::startTemperatureSampling()
{
    uint8_t modeConfig = readRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_TEMP_EN;

    writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

bool MAX30100::isTemperatureReady()
{
    return !(readRegister(MAX30100_REG_MODE_CONFIGURATION) & MAX30100_MC_TEMP_EN);
}

float MAX30100::retrieveTemperature()
{
    int8_t tempInteger = readRegister(MAX30100_REG_TEMPERATURE_DATA_INT);
    float tempFrac = readRegister(MAX30100_REG_TEMPERATURE_DATA_FRAC);

    return tempFrac * 0.0625 + tempInteger;
}

void MAX30100::shutdown()
{
    uint8_t modeConfig = readRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30100_MC_SHDN;

    writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

void MAX30100::resume()
{
    uint8_t modeConfig = readRegister(MAX30100_REG_MODE_CONFIGURATION);
    modeConfig &= ~MAX30100_MC_SHDN;

    writeRegister(MAX30100_REG_MODE_CONFIGURATION, modeConfig);
}

uint8_t MAX30100::getPartId()
{
    return readRegister(0xff);
}
