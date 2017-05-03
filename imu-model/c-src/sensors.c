/*
 * sensors.c
 *
 *  Created on: 12 апр. 2017 г.
 *      Author: developer
 */

#include <stdio.h>
#include <stdint.h>

#include "kinematic_unit.h"
#include "model/model.h"
#include "sensors.h"


void set_g_offset(float * g_offset)
{
	g_offset[0] = - G_VECT * STATE.f_XYZ[2][0];
	g_offset[1] = - G_VECT * STATE.f_XYZ[2][1];
	g_offset[2] = - G_VECT * STATE.f_XYZ[2][2];

}
