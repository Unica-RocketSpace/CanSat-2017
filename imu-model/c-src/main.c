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

	/*float M[3][3] = {	{	2,	3,	7	},
						{	-5,	4,	0	},
						{	1,	0,	-2	}	};
	float V[3] = {9, -4, 2};
	float S[3];

	solveSystemByKramer(*M, V, S);
	for (int i = 0; i < 3; i++)
	{
		printf("Solution %d: %f", i + 1, S[i]);
		printf("\n");
	}

	return 0;*/

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
	float ristart[3] = {0, 0, 100}, vistart[3] = {5, 0, 0}, ai[3] = {0, 0, -9.81}, oscfrec = 0.5, oscmagn = 1.5, rotfrec = 0.1;

	model_t * MODEL = model_init_freefal(ristart, vistart, ai, oscfrec, oscmagn, rotfrec);
	data_point_t DP;


	kinematicInit(ai, rotfrec, ristart, vistart);

	DP = model_evaluate(MODEL, time);
	float g_offset[3] = {0};

	while (DP.trueData.ri[2] > 0)
	{
		DP = model_evaluate(MODEL, time);

		set_g_offset(g_offset);

		STATE.aRelatedXYZ[0] = DP.obsData.af[0] + g_offset[0];
		STATE.aRelatedXYZ[1] = DP.obsData.af[1] + g_offset[1];
		STATE.aRelatedXYZ[2] = DP.obsData.af[2] + g_offset[2];

		STATE.gRelatedXYZ[0] = DP.obsData.wf[0];
		STATE.gRelatedXYZ[1] = DP.obsData.wf[1];
		STATE.gRelatedXYZ[2] = DP.obsData.wf[2];

		//определение угловых скоростей (в ИСК)
		RSC_to_ISC_recalc(STATE.gRelatedXYZ, STATE.w_XYZ);

		trajectoryConstruction(time);

		printf("time = %f s  ==================\n", time);
		printf("Accelerometer\n");
		printf("a_RSC_true: %f, %f, %f\n", DP.obsData.af[0], DP.obsData.af[1], DP.obsData.af[2]);
		printf("a_RSC_real: %f, %f, %f\n", STATE.aRelatedXYZ[0], STATE.aRelatedXYZ[1], STATE.aRelatedXYZ[2]);
		printf("Gyroscope\n");
		printf("w_RSC_true: %f, %f, %f\n", DP.obsData.wf[0], DP.obsData.wf[1], DP.obsData.wf[2]);
		printf("w_RSC_real: %f, %f, %f\n", STATE.gRelatedXYZ[0], STATE.gRelatedXYZ[1], STATE.gRelatedXYZ[2]);
		printf("Accelerations\n");
		printf("a_ISC_true: %f, %f, %f\n", DP.trueData.ai[0], DP.trueData.ai[1], DP.trueData.ai[2]);
		printf("a_ISC_real: %f, %f, %f\n", STATE.a_XYZ[0], STATE.a_XYZ[1], STATE.a_XYZ[2]);
		printf("Velocities\n");
		printf("v_ISC_true: %f, %f, %f\n", DP.trueData.vi[0], DP.trueData.vi[1], DP.trueData.vi[2]);
		printf("v_ISC_real: %f, %f, %f\n", STATE.v_XYZ[0], STATE.v_XYZ[1], STATE.v_XYZ[2]);
		printf("Translations\n");
		printf("s_ISC_true: %f, %f, %f\n", DP.trueData.ri[0], DP.trueData.ri[1], DP.trueData.ri[2]);
		printf("s_ISC_real: %f, %f, %f\n", STATE.s_XYZ[0], STATE.s_XYZ[1], STATE.s_XYZ[2]);
		printf("Angle velocities\n");
		printf("w_ISC_real: %f, %f, %f\n", STATE.w_XYZ[0], STATE.w_XYZ[1], STATE.w_XYZ[2]);
		printf("Rotation Matrix\n");
		printf("(%f / %f), (%f / %f), (%f / %f)\n",	DP.trueData.f_to_i[0][0], STATE.f_XYZ[0][0],
													DP.trueData.f_to_i[0][1], STATE.f_XYZ[0][1],
													DP.trueData.f_to_i[0][2], STATE.f_XYZ[0][2]);
		printf("(%f / %f), (%f / %f), (%f / %f)\n",	DP.trueData.f_to_i[1][0], STATE.f_XYZ[1][0],
													DP.trueData.f_to_i[1][1], STATE.f_XYZ[1][1],
													DP.trueData.f_to_i[1][2], STATE.f_XYZ[1][2]);
		printf("(%f / %f), (%f / %f), (%f / %f)\n",	DP.trueData.f_to_i[2][0], STATE.f_XYZ[2][0],
													DP.trueData.f_to_i[2][1], STATE.f_XYZ[2][1],
													DP.trueData.f_to_i[2][2], STATE.f_XYZ[2][2]);
		printf("\n");

		g_offset[0] = 0;
		g_offset[1] = 0;
		g_offset[2] = 0;
		time += 0.1;
	}

	printf("last ri[2]: %f", DP.trueData.ri[2]);




	return 0;
}

