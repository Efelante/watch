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

#include "max30100_filters.h"

float filterBuLp1_step(struct filterBuLp1 *ptr, float x) //class II
{
	ptr->v[0] = ptr->v[1];
	ptr->v[1] = (2.452372752527856026e-1 * x)
		+ (0.50952544949442879485 * ptr->v[0]);
	return
		(ptr->v[0] + ptr->v[1]);
}

void filterBuLp1_init(struct filterBuLp1 *ptr)
{
	ptr->v[0]=0.0;
}

void dcRemover_init(struct dcRemover *ptr, float alpha, float dcw)
{
	ptr->alpha = alpha;
	ptr->dcw = dcw;
}

float dcRemover_step(struct dcRemover *ptr, float x)
{
	float olddcw = ptr->dcw;
	ptr->dcw = (float) x + ptr->alpha * ptr->dcw;

	return ptr->dcw - olddcw;
}

float dcRemover_getDCW(struct dcRemover *ptr)
{
	return ptr->dcw;
}

