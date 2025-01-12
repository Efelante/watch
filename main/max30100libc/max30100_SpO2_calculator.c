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

#include <math.h>

#include "max30100_SpO2_calculator.h"

// SpO2 Look-up Table
// http://www.ti.com/lit/an/slaa274b/slaa274b.pdf
const uint8_t spO2LUT[43] = {100, 100, 100, 100, 99, 99, 99, 99,
							  99,  99,  98,  98, 98, 98, 98, 97, 
							  97,  97,  97,  97, 97, 96, 96, 96,
							  96,  96,  96,  95, 95, 95, 95, 95,
							  95,  94,  94,  94, 94, 94, 93, 93,
							  93,  93,  93};

void SpO2Calculator_init(struct SpO2Calculator *ptr)
{
	ptr->spO2LUT = spO2LUT;
    ptr->irACValueSqSum 	= 0;
    ptr->redACValueSqSum 	= 0;
    ptr->beatsDetectedNum 	= 0; 
    ptr->samplesRecorded 	= 0;
    ptr->spO2 				= 0; 
}

void SpO2Calculator_update(struct SpO2Calculator *ptr, float irACValue, float redACValue, bool beatDetected)
{
    ptr->irACValueSqSum += irACValue * irACValue;
    ptr->redACValueSqSum += redACValue * redACValue;
    ++ptr->samplesRecorded;

    if (beatDetected) {
        ++ptr->beatsDetectedNum;
        if (ptr->beatsDetectedNum == CALCULATE_EVERY_N_BEATS) {
            float acSqRatio = 100.0 * log(ptr->redACValueSqSum/ptr->samplesRecorded) / log(ptr->irACValueSqSum/ptr->samplesRecorded);
            uint8_t index = 0;

            if (acSqRatio > 66) {
                index = (uint8_t)acSqRatio - 66;
            } else if (acSqRatio > 50) {
                index = (uint8_t)acSqRatio - 50;
            }
            SpO2Calculator_reset(ptr);

            ptr->spO2 = ptr->spO2LUT[index];
        }
    }
}


void SpO2Calculator_reset(struct SpO2Calculator *ptr)
{
    ptr->samplesRecorded 	= 0;
    ptr->redACValueSqSum 	= 0;
    ptr->irACValueSqSum 	= 0;
    ptr->beatsDetectedNum 	= 0;
    ptr->spO2 				= 0;
}

uint8_t SpO2Calculator_getSpO2(struct SpO2Calculator *ptr)
{
    return ptr->spO2;
}
