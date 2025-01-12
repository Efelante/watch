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

//#include <Arduino.h>
#include "esp_timer.h" 
#include "esp_log.h" 

#define TAG "max30100"

#include "max30100_pulseoximeter.h"


static struct dcRemover irDCRemover;
static struct dcRemover redDCRemover;
static struct filterBuLp1 lpf;
static struct beatDetector beatDetector;
static struct SpO2Calculator spO2calculator;

void pulseOximeter_init(struct pulseOximeter *ptr, struct max30100 *max30100_ptr)
{
    ptr->state = PULSEOXIMETER_STATE_INIT;
	ptr->debuggingMode = PULSEOXIMETER_DEBUGGINGMODE_NONE;
    ptr->tsFirstBeatDetected = 0;
    ptr->tsLastBeatDetected = 0;
    ptr->tsLastBiasCheck = 0;
    ptr->tsLastCurrentAdjustment = 0;
	ptr->beatDetector = &beatDetector;
    ptr->irDCRemover = &irDCRemover;
    ptr->redDCRemover = &redDCRemover;
	ptr->lpf = &lpf;
    ptr->redLedCurrentIndex = (uint8_t) RED_LED_CURRENT_START;
    ptr->irLedCurrent = DEFAULT_IR_LED_CURRENT;
	ptr->spO2calculator = &spO2calculator;
	ptr->hrm = max30100_ptr;
    ptr->onBeatDetected = NULL;
}

static unsigned long millis()
{
	return (esp_timer_get_time() / 1000);
}


bool pulseOximeter_begin(struct pulseOximeter *ptr, PulseOximeterDebuggingMode debuggingMode)
{
    ptr->debuggingMode = debuggingMode;

    bool ready = max30100_begin(ptr->hrm);

    if (!ready) {
        if (ptr->debuggingMode != PULSEOXIMETER_DEBUGGINGMODE_NONE) {
    		ESP_LOGI(TAG, "Failed to initialize the HRM sensor");
        }
        return false;
    }

    //max30100_setMode(ptr->hrm, MAX30100_MODE_SPO2_HR);
    //max30100_setLedsCurrent(ptr->hrm, ptr->irLedCurrent, (LEDCurrent)ptr->redLedCurrentIndex);

	dcRemover_init(ptr->irDCRemover, DC_REMOVER_ALPHA, 0);
	dcRemover_init(ptr->redDCRemover, DC_REMOVER_ALPHA, 0);
	filterBuLp1_init(ptr->lpf);

	beatDetector_init(ptr->beatDetector);
	SpO2Calculator_init(ptr->spO2calculator);

    ptr->state = PULSEOXIMETER_STATE_IDLE;

    return true;
}

void pulseOximeter_update(struct pulseOximeter *ptr)
{
    max30100_update(ptr->hrm);

    pulseOximeter_checkSample(ptr);
    pulseOximeter_checkCurrentBias(ptr);
}

float pulseOximeter_getHeartRate(struct pulseOximeter *ptr)
{
    return beatDetector_getRate(ptr->beatDetector);	
}

uint8_t pulseOximeter_getSpO2(struct pulseOximeter *ptr)
{
    return SpO2Calculator_getSpO2(ptr->spO2calculator);	
}

uint8_t pulseOximeter_getRedLedCurrentBias(struct pulseOximeter *ptr)
{
    return ptr->redLedCurrentIndex;
}

void pulseOximeter_setOnBeatDetectedCallback(struct pulseOximeter *ptr, void (*cb)())
{
    ptr->onBeatDetected = cb;
}

void pulseOximeter_setIRLedCurrent(struct pulseOximeter *ptr, LEDCurrent irLedNewCurrent)
{
    ptr->irLedCurrent = irLedNewCurrent;
    max30100_setLedsCurrent(ptr->hrm, ptr->irLedCurrent, (LEDCurrent)ptr->redLedCurrentIndex);
}

void pulseOximeter_shutdown(struct pulseOximeter *ptr)
{
    max30100_shutdown(ptr->hrm);
}

void pulseOximeter_resume(struct pulseOximeter *ptr)
{
    max30100_resume(ptr->hrm);	
}

void pulseOximeter_checkSample(struct pulseOximeter *ptr)
{
    uint16_t rawIRValue, rawRedValue;

    // Dequeue all available samples, they're properly timed by the HRM
    while (max30100_getRawValues(ptr->hrm, &rawIRValue, &rawRedValue)) {
        float irACValue = dcRemover_step(ptr->irDCRemover, rawIRValue);
        float redACValue = dcRemover_step(ptr->redDCRemover, rawRedValue);

        // The signal fed to the beat detector is mirrored since the cleanest monotonic spike is below zero
        float filteredPulseValue = filterBuLp1_step(ptr->lpf, -irACValue);
        bool beatDetected = beatDetector_addSample(ptr->beatDetector, filteredPulseValue);

        if (beatDetector_getRate(ptr->beatDetector) > 0) {
            ptr->state = PULSEOXIMETER_STATE_DETECTING;
            SpO2Calculator_update(ptr->spO2calculator, irACValue, redACValue, beatDetected);
        } else if (ptr->state == PULSEOXIMETER_STATE_DETECTING) {
            ptr->state = PULSEOXIMETER_STATE_IDLE;
            SpO2Calculator_reset(ptr->spO2calculator);
        }

        switch (ptr->debuggingMode) {
            case PULSEOXIMETER_DEBUGGINGMODE_RAW_VALUES:
    			ESP_LOGI(TAG, "R:");
                ESP_LOGI(TAG, "%u", rawIRValue);
                ESP_LOGI(TAG, ",");
                ESP_LOGI(TAG, "%u", rawRedValue);
                break;

            case PULSEOXIMETER_DEBUGGINGMODE_AC_VALUES:
    			ESP_LOGI(TAG, "R:");
                ESP_LOGI(TAG, "%f", irACValue);
                ESP_LOGI(TAG, ",");
                ESP_LOGI(TAG, "%f", redACValue);
                break;

            case PULSEOXIMETER_DEBUGGINGMODE_PULSEDETECT:
    			ESP_LOGI(TAG, "R:");
                ESP_LOGI(TAG, "%f", filteredPulseValue);
                ESP_LOGI(TAG, ",");
                ESP_LOGI(TAG, "%f", beatDetector_getCurrentThreshold(ptr->beatDetector));
                break;

            default:
                break;
        }

        if (beatDetected && ptr->onBeatDetected) {
            //ptr->onBeatDetected();
        }
    }
}

void pulseOximeter_checkCurrentBias(struct pulseOximeter *ptr)
{
    // Follower that adjusts the red led current in order to have comparable DC baselines between
    // red and IR leds. The numbers are really magic: the less possible to avoid oscillations
    if (millis() - ptr->tsLastBiasCheck > CURRENT_ADJUSTMENT_PERIOD_MS) {
        bool changed = false;
        if (dcRemover_getDCW(ptr->irDCRemover) - dcRemover_getDCW(ptr->redDCRemover) > 70000 && ptr->redLedCurrentIndex < MAX30100_LED_CURR_50MA) {
            ++ptr->redLedCurrentIndex;
            changed = true;
        } else if (dcRemover_getDCW(ptr->redDCRemover) - dcRemover_getDCW(ptr->irDCRemover) > 70000 && ptr->redLedCurrentIndex > 0) {
            --ptr->redLedCurrentIndex;
            changed = true;
        }

        if (changed) {
            max30100_setLedsCurrent(ptr->hrm, ptr->irLedCurrent, (LEDCurrent)ptr->redLedCurrentIndex);
            ptr->tsLastCurrentAdjustment = millis();

            if (ptr->debuggingMode != PULSEOXIMETER_DEBUGGINGMODE_NONE) {
                ESP_LOGI(TAG, "I:");
                ESP_LOGI(TAG, "%i", ptr->redLedCurrentIndex);
            }
        }

        ptr->tsLastBiasCheck = millis();
    }
}
