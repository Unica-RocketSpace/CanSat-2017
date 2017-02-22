/*
 * servo_control.c
 *
 *  Created on: 11 февр. 2017 г.
 *      Author: developer
 */

#include <math.h>
#include "globals.h"


float setServoCosAngle () {
return	acos (((TARGET.X - STATE.sX) * (-STATE.fXZ1)
			+
		 (TARGET.Y - STATE.sY) * (-STATE.fYZ1)
		 	+
		 (0		   - STATE.sZ) * (-STATE.fZZ1))
			/
		sqrt(pow(TARGET.X - STATE.sX, 2)
				+
			 pow(TARGET.Y - STATE.sY, 2)
				+
			 pow( 		  - STATE.sY, 2)));

}
