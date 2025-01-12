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

#ifndef MAX30100_FILTERS_H
#define MAX30100_FILTERS_H

// http://www.schwietering.com/jayduino/filtuino/
// Low pass butterworth filter order=1 alpha1=0.1
// Fs=100Hz, Fc=6Hz
struct filterBuLp1
{
	float v[2];
	//float *(step)(float x) = filterBuLp1_step; //class II
};

float filterBuLp1_step(struct filterBuLp1 *ptr, float x); //class II
void filterBuLp1_init(struct filterBuLp1 *ptr);

// http://sam-koblenski.blogspot.de/2015/11/everyday-dsp-for-programmers-dc-and.html
struct dcRemover
{
	float alpha;
	float dcw;
};

void dcRemover_init(struct dcRemover *ptr, float alpha, float dcw);
float dcRemover_step(struct dcRemover *ptr, float x);
float dcRemover_getDCW(struct dcRemover *ptr);

#endif
