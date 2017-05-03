/*
 * sensors.h
 *
 *  Created on: 15 апр. 2017 г.
 *      Author: developer
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#define G_VECT	9.81

//установка смещения в виде проекции вектора g на связанные оси
/*	в model ускорение g вычитается	*/
void set_g_offset(float * g_offset);


#endif /* SENSORS_H_ */
