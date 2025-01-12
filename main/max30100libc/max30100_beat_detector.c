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

#include "esp_timer.h" 

#include "max30100_beat_detector.h"

#ifndef min
#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#endif

static unsigned long millis()
{
	return (esp_timer_get_time() / 1000);
}

void beatDetector_init(struct beatDetector *ptr)
{
    ptr->state = BEATDETECTOR_STATE_INIT;
    ptr->threshold = BEATDETECTOR_MIN_THRESHOLD;
    ptr->beatPeriod = 0;
    ptr->lastMaxValue = 0;
    ptr->tsLastBeat = 0;
}

bool beatDetector_addSample(struct beatDetector *ptr, float sample)
{
    return beatDetector_checkForBeat(ptr, sample);
}

float beatDetector_getRate(struct beatDetector *ptr)
{
    if (ptr->beatPeriod != 0) {
        return 1 / ptr->beatPeriod * 1000 * 60;
    } else {
        return 0;
    }
}

float beatDetector_getCurrentThreshold(struct beatDetector *ptr)
{
    return ptr->threshold;
}

bool beatDetector_checkForBeat(struct beatDetector *ptr, float value)
{
    bool beatDetected = false;

    switch (ptr->state) {
        case BEATDETECTOR_STATE_INIT:
            if (millis() > BEATDETECTOR_INIT_HOLDOFF) {
                ptr->state = BEATDETECTOR_STATE_WAITING;
            }
            break;

        case BEATDETECTOR_STATE_WAITING:
            if (value > ptr->threshold) {
                ptr->threshold = min(value, BEATDETECTOR_MAX_THRESHOLD);
                ptr->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }

            // Tracking lost, resetting
            if (millis() - ptr->tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY) {
                ptr->beatPeriod = 0;
                ptr->lastMaxValue = 0;
            }

            beatDetector_decreaseThreshold(ptr);
            break;

        case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
            if (value < ptr->threshold) {
                ptr->state = BEATDETECTOR_STATE_MAYBE_DETECTED;
            } else {
                ptr->threshold = min(value, BEATDETECTOR_MAX_THRESHOLD);
            }
            break;

        case BEATDETECTOR_STATE_MAYBE_DETECTED:
            if (value + BEATDETECTOR_STEP_RESILIENCY < ptr->threshold) {
                // Found a beat
                beatDetected = true;
                ptr->lastMaxValue = value;
                ptr->state = BEATDETECTOR_STATE_MASKING;
                float delta = millis() - ptr->tsLastBeat;
                if (delta) {
                    ptr->beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta +
                            (1 - BEATDETECTOR_BPFILTER_ALPHA) * ptr->beatPeriod;
                }

                ptr->tsLastBeat = millis();
            } else {
                ptr->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }
            break;

        case BEATDETECTOR_STATE_MASKING:
            if (millis() - ptr->tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF) {
                ptr->state = BEATDETECTOR_STATE_WAITING;
            }
            beatDetector_decreaseThreshold(ptr);
            break;
    }

    return beatDetected;
}

void beatDetector_decreaseThreshold(struct beatDetector *ptr)
{
    // When a valid beat rate readout is present, target the
    if (ptr->lastMaxValue > 0 && ptr->beatPeriod > 0) {
        ptr->threshold -= ptr->lastMaxValue * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) /
                (ptr->beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);
    } else {
        // Asymptotic decay
        ptr->threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
    }

    if (ptr->threshold < BEATDETECTOR_MIN_THRESHOLD) {
        ptr->threshold = BEATDETECTOR_MIN_THRESHOLD;
    }
}
