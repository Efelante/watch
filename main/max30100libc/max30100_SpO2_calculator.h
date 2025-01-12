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

#ifndef MAX30100_SPO2CALCULATOR_H
#define MAX30100_SPO2CALCULATOR_H

#include <stdint.h>
#include <stdbool.h>

#define CALCULATE_EVERY_N_BEATS         3

struct SpO2Calculator 
{
    const uint8_t *spO2LUT;
    float irACValueSqSum;
    float redACValueSqSum;
    uint8_t beatsDetectedNum;
    uint32_t samplesRecorded;
    uint8_t spO2;
};

void 	SpO2Calculator_init(struct SpO2Calculator *ptr);
void 	SpO2Calculator_update(struct SpO2Calculator *ptr, float irACValue, float redACValue, bool beatDetected);
void 	SpO2Calculator_reset(struct SpO2Calculator *ptr);
uint8_t SpO2Calculator_getSpO2(struct SpO2Calculator *ptr);

#endif
