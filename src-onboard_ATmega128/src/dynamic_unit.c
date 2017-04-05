/*
 * servo_control.c
 *
 *  Created on: 11 февр. 2017 г.
 *      Author: developer
 */

#include <math.h>
#include <avr/io.h>

#include "timer.h"
#include "dynamic_unit.h"


float WheelSpeed_current = 0;


void dynamicInit()
{

}


float calculateWheelSpeed(float angVelocityX, float angVelocityY, float angVelocityZ)
{
	return (DEV_INERT_MOM / WHEEL_INERT_MOM) * angVelocityZ + WheelSpeed_current;
}


float calculateServoAngle(	float positionX,	float positionY,	float positionZ,
							float cosAngleX_Z1,	float cosAngleY_Z1,	float cosAngleZ_Z1,
							float targetX,		float targetY)
{
	float ServoAngle = acos (((targetX - positionX) * (-cosAngleX_Z1)
						+ (targetY - positionY) * (-cosAngleY_Z1)
						+ (0 - positionZ) * (-cosAngleZ_Z1))
						/ sqrt(	pow(targetX - positionX, 2)
								+ pow(targetY - positionY, 2)
								+ pow(-positionY, 2)));

	return ServoAngle;
}


void setWheelSpeed(float WheelSpeed)
{
	if (WheelSpeed > 0) // TURN RIGHT
		{
			DRV_PORT |= (1 << WH1_C);
			DRV_PORT |= (0 << WH1_D);
		}
	else		    // TURN LEFT
		{
			DRV_PORT |= (0 << WH1_C);
			DRV_PORT |= (1 << WH1_D);
		}

	WHEEL_SPEED_REG = (int)((WheelSpeed / MAX_WHEEL_SPEED) * (3 / 3.3) * 0xFFFF);
}


void setServoAngle(float ServoAngle)
{
	if (ServoAngle > MAX_SERVO_ANGLE) ServoAngle = MAX_SERVO_ANGLE;
	if (ServoAngle > MIN_SERVO_ANGLE) ServoAngle = MIN_SERVO_ANGLE;
	SERVO_ANGLE_REG = 3277 * ((ServoAngle - MIN_SERVO_ANGLE) / (MAX_SERVO_ANGLE - MIN_SERVO_ANGLE) + 1);
}
