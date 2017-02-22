/*
 * wheel_control.h
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */

#ifndef WHEEL_CONTROL_H_
#define WHEEL_CONTROL_H_

void wheel_init();
void setSHIMfreq(int SHIMn, float speed);
void wheel_run(int driver_number, float speed);

float globalWheelSpeed[3];


#endif /* WHEEL_CONTROL_H_ */
