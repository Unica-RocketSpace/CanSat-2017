/*
 * ParabolaModel.h
 *
 *  Created on: 6 апр. 2017 г.
 *      Author: snork
 */

#ifndef CPP_FREEFALLOSCILLATINGMODEL_H_
#define CPP_FREEFALLOSCILLATINGMODEL_H_

#include "glm/gtc/constants.hpp"

#include "Model.h"
#include "structs.h"

namespace Model
{

	class FreeFallOscillatingModel: public Model
	{
	public:
		//! Параметры:
		/*!
		 *  ristart - начальное положение объекта в ИСК
		 *  vistart - начальная скорость объекта в ИСК
		 *  ai - постоянное ускорение, действующее в системе в ИСК
		 *  osc_freq - частота колебаний аппарата вокруг оси Y (в Гц)
		 *  osc_magn - амплитуда колебаний аппарата вокруг оси Y (в рад)
		 *  rot_freq - частота вращения аппарата вокруг оси Z (в Гц) */
		FreeFallOscillatingModel(glm::vec3 ristart, glm::vec3 vistart, glm::vec3 ai,
				float osc_freq, float osc_magn, float rot_freq)
			: _ri_start(ristart), _vi_start(vistart), _ai(ai),
			  _osc_freq(osc_freq), _osc_magn(osc_magn), _rot_freq(rot_freq)
		{}

	protected:
		virtual InertialData _do_evaluate(time_t tsince) override
		{
			InertialData retval;

			// по положению ценра масс - простое равноускоренное движение
			retval.ri = _ri_start
					+ (_vi_start * tsince)
					+ (_ai * tsince * tsince) / 2.0f;

			retval.vi = _vi_start + _ai * tsince;

			retval.ai = _ai;

			// по ориентации - сделаем два вращения.
			// первое - будем качать аппарат относительно оси Y на циклический угор
			const float first_rot_angle = _osc_magn * sin( _osc_freq / 2.0f / glm::pi<float>() * tsince);
			glm::quat first_rot = glm::angleAxis(first_rot_angle , glm::vec3(0.0f, 1.0f, 0.0f));

			// второе - будем равномерно вращать аппарат вокруг его оси Z
			// вычтем из tsince прошедшие полные периоды оборота
			const float second_rot_angle = (_rot_freq * 2 * glm::pi<float>() * tsince);
			glm::quat second_rot = glm::angleAxis(second_rot_angle, glm::vec3(0.0f, 0.0f, 1.0f));


			// комбинируем вращения
			retval.f_to_i = second_rot * first_rot;

			/*
			std::cout << second_rot_angle << "...."
					<< retval.f_to_i * glm::vec3(1, 0, 0) * glm::conjugate(retval.f_to_i) << std::endl;
            */

			return retval;
		}

	private:
		glm::vec3 _ri_start;
		glm::vec3 _vi_start;
		glm::vec3 _ai;

		float _osc_freq;
		float _osc_magn;
		float _rot_freq;
	};
}

#endif /* CPP_FREEFALLOSCILLATINGMODEL_H_ */
