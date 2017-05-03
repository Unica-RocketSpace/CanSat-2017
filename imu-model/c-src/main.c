/*
 * main.c
 *
 *  Created on: 5 апр. 2017 г.
 *      Author: snork
 */

#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include <sofa/sofam.h>

#include "../model/model.h"
#include <sofa/sofa.h>
#include "kinematic_unit.h"
#include "sensors.h"


state STATE;

int main()
{

	/*float rstart[3] = {0,0,100};
	float vstart[3] = {0,0,0};
	float a[3]		= {0,0,-9.8};
	float oscfreq = 0; //0.02;
	float oscmagn = DD2R * 45.0f;
	float rotfreq = 0.1;

	model_t * model = model_init_freefal(rstart, vstart, a, oscfreq, oscmagn, rotfreq);

	for (float tsince = 0.f ; tsince < 10.f; tsince += 1.0f)
	{
		data_point_t dp = model_evaluate(model, tsince);

		printf("tri[% 10.4f, %10.4f, %10.4f]", dp.trueData.ri[0], dp.trueData.ri[1], dp.trueData.ri[2]);
		printf(" ");
		printf("tvi[% 10.4f, %10.4f, %10.4f]", dp.trueData.vi[0], dp.trueData.vi[1], dp.trueData.vi[2]);
		printf(" ");
		printf("tai[% 10.4f, %10.4f, %10.4f]", dp.trueData.ai[0], dp.trueData.ai[1], dp.trueData.ai[2]);
		printf("\n");


		printf("ofa[% 10.4f, %10.4f, %10.4f]", dp.obsData.af[0], dp.obsData.af[1], dp.obsData.af[2]);
		printf(" ");
		printf("ofw[% 10.4f, %10.4f, %10.4f]", dp.obsData.wf[0], dp.obsData.wf[1], dp.obsData.wf[2]);
		printf(" ");
		*/

		/*
		float x_fixed[3] = {1, 0, 0};
		float x_fixed_in_ics[3];
		//iauTr(dp.trueData.f_to_i, dp.trueData.f_to_i);
		iauRxp(dp.trueData.f_to_i, x_fixed, x_fixed_in_ics);
		printf("xfi[% 10.4f, %10.4f, %10.4f]", x_fixed_in_ics[0], x_fixed_in_ics[1], x_fixed_in_ics[2]);
		*/
/*
		printf("\n");
	}
*/

	float time = 0;
	float ristart[3] = {0, 0, 100}, vistart[3] = {5, 0, 0}, ai[3] = {0, 0, -9.81}, oscfrec = 0, oscmagn = 0, rotfrec = 0.1;

	model_t * MODEL = model_init_freefal(ristart, vistart, ai, oscfrec, oscmagn, rotfrec);
	data_point_t DP;


	kinematicInit(ai, rotfrec, ristart);

	DP = model_evaluate(MODEL, time);
	float g_offset[3] = {0};

	while (DP.trueData.ri[2] > 0)
	{
		DP = model_evaluate(MODEL, time);

		set_g_offset(g_offset);

		STATE.aRelatedXYZ[0] = DP.obsData.af[0] + g_offset[0];
		STATE.aRelatedXYZ[1] = DP.obsData.af[1] + g_offset[1];
		STATE.aRelatedXYZ[2] = DP.obsData.af[2] + g_offset[2];

		STATE.w_XYZ[0] = DP.obsData.wf[0];
		STATE.w_XYZ[1] = DP.obsData.wf[1];
		STATE.w_XYZ[2] = DP.obsData.wf[2];

		trajectoryConstruction(time);

		printf("af[0]: %f\n", DP.obsData.af[0]);
		printf("af[1]: %f\n", DP.obsData.af[1]);
		printf("af[2]: %f\n", DP.obsData.af[2]);

		printf("wf[0]: %f\n", DP.obsData.wf[0]);
		printf("wf[1]: %f\n", DP.obsData.wf[1]);
		printf("wf[2]: %f\n", DP.obsData.wf[2]);

		printf("STATE.a_XYZ[0]: %f\n", STATE.a_XYZ[0]);
		printf("STATE.a_XYZ[1]: %f\n", STATE.a_XYZ[1]);
		printf("STATE.a_XYZ[2]: %f\n", STATE.a_XYZ[2]);

		printf("ri[0]: %f\n", DP.trueData.ri[0]);
		printf("ri[1]: %f\n", DP.trueData.ri[1]);
		printf("ri[2]: %f\n", DP.trueData.ri[2]);

		printf("STATE.s_XYZ[0]: %f\n", STATE.s_XYZ[0]);
		printf("STATE.s_XYZ[1]: %f\n", STATE.s_XYZ[1]);
		printf("STATE.s_XYZ[2]: %f\n", STATE.s_XYZ[2]);


		printf("\n\n");

		g_offset[0] = 0;
		g_offset[1] = 0;
		g_offset[2] = 0;
		time += 1;
	}

	printf("last ri[2]: %f", DP.trueData.ri[2]);




	return 0;
}

