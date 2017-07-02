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
	DRV_DDR |= (1 << WH1_C) | (1 << WH1_D);
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
	if (WheelSpeed > 0)					// TURN RIGHT
		{
			DRV_PORT |= (1 << WH1_C);
			DRV_PORT &= ~(1 << WH1_D);
		}
	else if (WheelSpeed < 0)		    // TURN LEFT
		{
			DRV_PORT &= ~(1 << WH1_C);
			DRV_PORT |= (1 << WH1_D);
			WheelSpeed = - WheelSpeed;
		}
	else
	{
		DRV_PORT |= (1 << WH1_C);		//FAST MOTOR STOP (это не так)
		DRV_PORT |= (1 << WH1_D);
	}

	WHEEL_SPEED_REG = (round)((WheelSpeed / MAX_WHEEL_SPEED)/* * (3 / 3.3)*/ * 20000);
}


void setServoAngle(float ServoAngle)
{
	float AngleRatio = (ServoAngle + 90) / 180; //FIXME: заменить 90 и 180 на M_PI_2 и M_PI соответственно
	if (AngleRatio > 1) AngleRatio = 1;
	if (AngleRatio < 0) AngleRatio = 0;

	SERVO_ANGLE_REG = (round)(AngleRatio * (MAX_SERVO_ANGLE_REG - MIN_SERVO_ANGLE_REG)) + MIN_SERVO_ANGLE_REG;
}
