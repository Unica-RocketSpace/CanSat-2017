/*
 * Model.h
 *
 *  Created on: 6 апр. 2017 г.
 *      Author: snork
 */

#ifndef CPP_MODEL_H_
#define CPP_MODEL_H_

#include <tuple>

#include "structs.h"


#include <iostream>
#include "glm/gtx/io.hpp"
#include "glm/gtx/quaternion.hpp"

namespace Model
{

	class Model
	{
	public:

		Model() = default;
		virtual ~Model() = default;

		std::tuple<InertialData, ObservedData> evaluate(time_t tsince)
		{
			// инерциалку считает модель
			InertialData idata = _do_evaluate(tsince);

			// а остальное мы пересчитываем из инерциалки
			ObservedData odata;
			// с ускорениями все просто - добавляем к ним мнимое g, которое обеспечивается гравитацией в нашей системе
			const glm::vec3 g(0.0f, 0.0f, -9.81f);
			odata.af = glm::conjugate(idata.f_to_i) * (idata.ai - g) * idata.f_to_i;

			// с угловыми скоростями - её сперва требуется найти
			// согласно https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
			// её можно найти по формуле
			// w = 2 * dq/dt * 1/q = 2 * dq/dt * ~q;
			const glm::quat q = idata.f_to_i;
			// dq/dt найдем конечной производной
			const float delta_t = 0.001;
			const glm::quat next_q = _do_evaluate(tsince + delta_t).f_to_i;
			const glm::quat dq_dt = (next_q + (-q)) / delta_t; // Почему-то в GLM нет оператора -. Поэтому nextq + (-q)

			glm::quat wq = (2.0f * dq_dt * glm::conjugate(q));
			glm::vec3 w = glm::vec3(wq.x, wq.y, wq.z);

			odata.wf = w; // FIXME: нужено  ли тут *(-1)? поидее это скорость с которой движется ССК относитель ИСК

			// все, расчет закончен
			return std::make_pair(idata, odata);
		}


	private:
		virtual InertialData _do_evaluate(time_t tsince) = 0;
	};

}

#endif /* CPP_MODEL_H_ */
