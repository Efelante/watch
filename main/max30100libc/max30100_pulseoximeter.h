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

#ifndef MAX30100_PULSEOXIMETER_H
#define MAX30100_PULSEOXIMETER_H

#define SAMPLING_FREQUENCY                  100
#define CURRENT_ADJUSTMENT_PERIOD_MS        500
#define DEFAULT_IR_LED_CURRENT              MAX30100_LED_CURR_50MA
#define RED_LED_CURRENT_START               MAX30100_LED_CURR_27_1MA
#define DC_REMOVER_ALPHA                    0.95

#include <stdint.h>

#include "max30100.h"
#include "max30100_beat_detector.h"
#include "max30100_filters.h"
#include "max30100_SpO2_calculator.h"

typedef enum PulseOximeterState {
    PULSEOXIMETER_STATE_INIT,
    PULSEOXIMETER_STATE_IDLE,
    PULSEOXIMETER_STATE_DETECTING
} PulseOximeterState;

typedef enum PulseOximeterDebuggingMode {
    PULSEOXIMETER_DEBUGGINGMODE_NONE,
    PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES,
    PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES,
    PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT
} PulseOximeterDebuggingMode;

struct pulseOximeter 
{
    PulseOximeterState state;
    PulseOximeterDebuggingMode debuggingMode;
    uint32_t tsFirstBeatDetected;
    uint32_t tsLastBeatDetected;
    uint32_t tsLastBiasCheck;
    uint32_t tsLastCurrentAdjustment;
    struct beatDetector *beatDetector;
    struct dcRemover *irDCRemover;
    struct dcRemover *redDCRemover;
    struct filterBuLp1 *lpf;
    uint8_t redLedCurrentIndex;
    LEDCurrent irLedCurrent;
    struct SpO2Calculator *spO2calculator;
    struct max30100 *hrm;

    void (*onBeatDetected)();
};

void pulseOximeter_init(struct pulseOximeter *ptr, struct max30100 *max30100_ptr);
bool pulseOximeter_begin(struct pulseOximeter *ptr, PulseOximeterDebuggingMode debuggingMode);
void pulseOximeter_update(struct pulseOximeter *ptr);
float pulseOximeter_getHeartRate(struct pulseOximeter *ptr);
uint8_t pulseOximeter_getSpO2(struct pulseOximeter *ptr);
uint8_t pulseOximeter_getRedLedCurrentBias(struct pulseOximeter *ptr);
void pulseOximeter_setOnBeatDetectedCallback(struct pulseOximeter *ptr, void (*cb)());
void pulseOximeter_setIRLedCurrent(struct pulseOximeter *ptr, LEDCurrent irLedNewCurrent);
void pulseOximeter_shutdown(struct pulseOximeter *ptr);
void pulseOximeter_resume(struct pulseOximeter *ptr);
void pulseOximeter_checkSample(struct pulseOximeter *ptr);
void pulseOximeter_checkCurrentBias(struct pulseOximeter *ptr);

#endif
