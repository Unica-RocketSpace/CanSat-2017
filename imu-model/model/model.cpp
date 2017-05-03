/*
 * model.cc
 *
 *  Created on: 7 апр. 2017 г.
 *      Author: snork
 */


extern "C" {
#include "model.h"
}

#include <stdlib.h>


#include "cpp/glm/glm.hpp"
#include "cpp/glm/gtc/type_ptr.hpp"

#include "cpp/FreeFallOscillatingModel.h"

struct model_t
{
	Model::Model * impl = nullptr;
};


model_t * model_init_freefal(float * ristart, float * vistart, float * ai, float oscfreq, float oscqmagn, float rotfreq)
{
	model_t * retval = (model_t*)malloc(sizeof (model_t));
	retval->impl = new Model::FreeFallOscillatingModel(
			glm::make_vec3(ristart), glm::make_vec3(vistart), glm::make_vec3(ai),
			oscfreq, oscqmagn, rotfreq
		);

	return retval;
}


data_point_t model_evaluate(model_t * self, float tsince)
{
	auto cpp_retval = self->impl->evaluate(tsince);
	Model::InertialData & idata = std::get<0>(cpp_retval);
	Model::ObservedData & odata = std::get<1>(cpp_retval);

	static_assert(std::is_same<glm::vec3::value_type, float>::value, "glm value type is not float");

	// сперва самое простое - перекидываем векторы в сишную структуру
	data_point_t retval;
	std::memcpy(retval.trueData.ri, glm::value_ptr(idata.ri), 3*sizeof(glm::vec3::value_type));
	std::memcpy(retval.trueData.vi, glm::value_ptr(idata.vi), 3*sizeof(glm::vec3::value_type));
	std::memcpy(retval.trueData.ai, glm::value_ptr(idata.ai), 3*sizeof(glm::vec3::value_type));

	std::memcpy(retval.obsData.af, glm::value_ptr(odata.af), 3*sizeof(glm::vec3::value_type));
	std::memcpy(retval.obsData.wf, glm::value_ptr(odata.wf), 3*sizeof(glm::vec3::value_type));

	// теперь перекидываем кватернион в матрицу
	glm::mat3 f_to_i = glm::mat3_cast(idata.f_to_i);
	// а вот тут делаем грязный хак.
	// предполагаем что в сишной матрице строки и столбцы расположены так же как glm
	// и тупо копируем блок памяти матрицы
	std::memcpy(retval.trueData.f_to_i, glm::value_ptr(f_to_i), 3*3*sizeof(glm::mat3::value_type));

	return retval;
}


void model_deinit(model_t * self)
{
	delete self->impl;
	free(self);
}

