/*
 * DataPoint.h
 *
 *  Created on: 6 апр. 2017 г.
 *      Author: snork
 */

#ifndef CPP_STRUCTS_H_
#define CPP_STRUCTS_H_

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace Model
{
	typedef float time_t;

	struct InertialData
	{
		glm::vec3 ri;
		glm::vec3 vi;
		glm::vec3 ai;
		glm::quat f_to_i;
	};

	struct ObservedData
	{
		glm::vec3 af;
		glm::vec3 wf;
	};

}


#endif /* CPP_STRUCTS_H_ */
