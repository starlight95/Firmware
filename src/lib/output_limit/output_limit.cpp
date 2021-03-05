/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "output_limit.h"

#include <px4_platform_common/defines.h>
#include <math.h>
#include <stdbool.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <stdlib.h>

#define PROGRESS_INT_SCALING	10000

void output_limit_init(output_limit_t *limit)
{
	limit->state = OUTPUT_LIMIT_STATE_INIT;
	limit->time_armed = 0;
	limit->ramp_up = true;
}

void output_limit_calc(const bool armed, const bool pre_armed, const unsigned num_channels, const uint16_t reverse_mask,
		       const uint16_t *disarmed_output, const uint16_t *min_output, const uint16_t *max_output,
		       const float *output, uint16_t *effective_output, output_limit_t *limit)
{
    // _v_att_sub.update(&_v_att);
	// matrix::Eulerf att =matrix::Quatf(_v_att.q);
	/* first evaluate state changes */
	switch (limit->state) {
	case OUTPUT_LIMIT_STATE_INIT:

		if (armed) {

			/* set arming time for the first call */
			if (limit->time_armed == 0) {
				limit->time_armed = hrt_absolute_time();
			}

			if (hrt_elapsed_time(&limit->time_armed) >= INIT_TIME_US) {
				limit->state = OUTPUT_LIMIT_STATE_OFF;
			}
		}

		break;

	case OUTPUT_LIMIT_STATE_OFF:
		if (armed) {
			if (limit->ramp_up) {
				limit->state = OUTPUT_LIMIT_STATE_RAMP;

			} else {
				limit->state = OUTPUT_LIMIT_STATE_ON;
			}

			/* reset arming time, used for ramp timing */
			limit->time_armed = hrt_absolute_time();
		}

		break;

	case OUTPUT_LIMIT_STATE_RAMP:
		if (!armed) {
			limit->state = OUTPUT_LIMIT_STATE_OFF;

		} else if (hrt_elapsed_time(&limit->time_armed) >= RAMP_TIME_US) {
			limit->state = OUTPUT_LIMIT_STATE_ON;
		}

		break;

	case OUTPUT_LIMIT_STATE_ON:
		if (!armed) {
			limit->state = OUTPUT_LIMIT_STATE_OFF;
		}

		break;

	default:
		break;
	}

	/* if the system is pre-armed, the limit state is temporarily on,
	 * as some outputs are valid and the non-valid outputs have been
	 * set to NaN. This is not stored in the state machine though,
	 * as the throttle channels need to go through the ramp at
	 * regular arming time.
	 * 如果系统已预先启用，则限制状态暂时打开，因为有些输出是有效的，而无效的输出被设置为NaN。
	 * 不过，这并不存储在状态机中(需要在外部调用中被赋予)，因为油门通道需要在正常的启用时间通过斜坡加速打开
	 */

	unsigned local_limit_state = limit->state;

	if (pre_armed) {
		local_limit_state = OUTPUT_LIMIT_STATE_ON;
	}

	unsigned progress;


	/* then set effective_output based on state 基于状态设定有效输出*/
	switch (local_limit_state) {
	case OUTPUT_LIMIT_STATE_OFF:
	case OUTPUT_LIMIT_STATE_INIT:
		for (unsigned i = 0; i < num_channels; i++) {
			effective_output[i] = disarmed_output[i];
		}

		break;

	case OUTPUT_LIMIT_STATE_RAMP: {
			hrt_abstime diff = hrt_elapsed_time(&limit->time_armed);

			progress = diff * PROGRESS_INT_SCALING / RAMP_TIME_US;

			if (progress > PROGRESS_INT_SCALING) {
				progress = PROGRESS_INT_SCALING;
			}

			for (unsigned i = 0; i < num_channels; i++) {

				float control_value = output[i];

				/* check for invalid / disabled channels */
				if (!PX4_ISFINITE(control_value)) {
					effective_output[i] = disarmed_output[i];
					continue;
				}

				uint16_t ramp_min_output;

				/* if a disarmed output value was set, blend between disarmed and min */
				if (disarmed_output[i] > 0) {

					/* safeguard against overflows */
					unsigned disarmed = disarmed_output[i];

					if (disarmed > min_output[i]) {
						disarmed = min_output[i];
					}

					unsigned disarmed_min_diff = min_output[i] - disarmed;
					ramp_min_output = disarmed + (disarmed_min_diff * progress) / PROGRESS_INT_SCALING;

				} else {

					/* no disarmed output value set, choose min output */
					ramp_min_output = min_output[i];
				}

				if (reverse_mask & (1 << i)) {
					control_value = -1.0f * control_value;
				}

				effective_output[i] = control_value * (max_output[i] - ramp_min_output) / 2 + (max_output[i] + ramp_min_output) / 2;

				/* last line of defense against invalid inputs */
				if (effective_output[i] < ramp_min_output) {
					effective_output[i] = ramp_min_output;

				} else if (effective_output[i] > max_output[i]) {
					effective_output[i] = max_output[i];
				}
			}
		}
		break;

	case OUTPUT_LIMIT_STATE_ON:

		for (unsigned i = 0; i < num_channels; i++) {

			float control_value = output[i];

			/* check for invalid / disabled channels */
			if (!PX4_ISFINITE(control_value)) {
				effective_output[i] = disarmed_output[i];
				continue;
			}

			if (reverse_mask & (1 << i)) {
				control_value = -1.0f * control_value;
			}

			effective_output[i] = control_value * (max_output[i] - min_output[i]) / 2 + (max_output[i] + min_output[i]) / 2;

			/* last line of defense against invalid inputs */
			if (effective_output[i] < min_output[i]) {
				effective_output[i] = min_output[i];

			} else if (effective_output[i] > max_output[i]) {
				effective_output[i] = max_output[i];
			}

		}

		break;

	default:
		break;
	}

}

void output_correct(uint16_t *effective_output)
{
	if(effective_output[2] <= 1100)
	{
		return;
	}else{
		double standard_output = effective_output[2];
	        double standard_thrust = 0.00068590*standard_output*standard_output-1.4284*standard_output+740.6810;
                double corrcect_output = (1.1824+sqrt(1.1824*1.1824 -4*0.00062082*(542.3824-standard_thrust)))/(2*0.00062082);

                effective_output[3] = (uint16_t) corrcect_output;
	}

}

void output_reorder(const float *output, float *output_change,bool water_flag,int k_turn)
{
        int k = k_turn;
	for (unsigned i = 0;i < 8;i++)
	{
		output_change[i] = output[i];
	}


	if(!water_flag)
	{
		output_change[3] = output[5];
		output_change[2] = output[7];
	}
	else
	{
		float max_1 = output[5];
		float max_2 = output[7];
		float min = -1.0f;

                if(k>=0&&k<20)
		{
			output_change[5] = (max_1-min)*k/20+min;
		        // output_change[3] = NAN;

		        output_change[7] = (max_2-min)*k/20+min;
		        // output_change[2] = NAN;

		}else
		{
			output_change[5] = output[5];
		        // output_change[3] = NAN;

		        output_change[7] = output[7];
		        // output_change[2] = NAN;
		}

	}

}

void output_add(uint16_t *output,const float sin_roll,const float throttle)
{


        if(output[6] <= 1100 && output[4]<=1100)
	{
		return;
	}else{
		if(sin_roll>=0)
	        {
			output[4] = output[4]+840*(1-throttle)*sin_roll;//840-- total pwm range.
			output[6] = output[6];
		}
		if(sin_roll<0)
		{
			output[4] = output[4];
			output[6] = output[6]-840*(1-throttle)*(sin_roll);
	        }
	}


}

void output_add_with_roll(uint16_t *output,const float sin_roll,float throttle)
{
	if(output[6] <= 1100 && output[4]<=1100)
	{
		return;
	}else{
		if(sin_roll>=0)
	        {
			//840-- total pwm range.
			output[4] = output[4]+84;
			output[6] = 1100+840*(throttle+0.1f);
		}
		if(sin_roll<0)
		{
			output[4] = 1100+840*(throttle+0.1f);
			output[6] = output[6]+84;

	        }
	}
}

void output_add_no_roll(uint16_t *output,const float sin_roll,float throttle)
{

        if(output[6] <= 1100 && output[4]<=1100)
	{
		return;
	}else{
		if(sin_roll>=0)
	        {
			output[4] = output[4]+126.0f+840.0f*(1-throttle-0.15f)*(sin_roll);//840-- total pwm range.
			output[6] = output[6]+126.0f;
			output[3] = output[3]-840.0f*throttle*(sin_roll);
		}
		if(sin_roll<0)
		{
			output[2] = output[2]+840.0f*throttle*(sin_roll);
			output[4] = output[4]+126.0f;
			output[6] = output[6]+126.0f-840.0f*(1-throttle-0.15f)*(sin_roll);
	        }
	}
}

void output_correct_air(uint16_t *output,const float sin_roll,float throttle)
{

        if(output[6] <= 1100 && output[4]<=1100)
	{
		return;
	}else{
		if(sin_roll>=0)
	        {
			output[5] = output[5]-840*throttle*2*abs(sin_roll);
		}
		if(sin_roll<0)
		{
			output[7] = output[7]+840*throttle*2*sin_roll;

	        }
	}
}
void output_correct_back(uint16_t *output,const float sin_roll)
{
	double rpm1 = 0.0;
	double rpm2 = 0.0;
	uint16_t output1 = 0;
	uint16_t output2 = 0;
	double force = 0.0;
	float distance = 0.95f*(float)tan(asin(sin_roll)); // 不知道能不能用。
	int piece_of_dis = floor(distance/2.0f);
	double a;
	double b;
	if(sin_roll>=0)
	{
		output1 = output[2];
	}else
	{
		output1 = output[3];
	}

	rpm1 = 0.0235*output1*output1-46.1221*output1+2.4613e04;
	force = -1.060786e-07*rpm1*rpm1+3.513202e-03*rpm1;

	switch (piece_of_dis)
	{
	case 0:
                a = -1.060786e-07;b = 3.513202e-03;
		break;
	case 1:
		a = -2.186879e-08;b = 2.007845e-03;
		break;
	case 2:
		a = 3.269230e-08;b = 1.084307e-03;
		break;
	case 3:
		a = 5.514003e-08;b = 6.229734e-03;
		break;
	case 4:
	case 5:
		a = -5.499961e-09;b = 1.767934e-03;
		break;
	case 6:
		a = 4.043999e-08;b = 1.119353e-03;
		break;
	case 7:
		a = 9.371167e-09;b = 2.027758e-03;
		break;
	case 8:
	case 9:
		a = 5.643773e-09;b = 2.348638e-03;
		break;
	case 10:
		a = 5.995059e-08;b = 1.389641e-03;
		break;
	case 11:
		a = 4.456715e-08;b = 1.851485e-03;
		break;
	case 12:
		a = 7.326941e-08;b = 1.350430e-03;
		break;
	case 13:
		a = 9.711139e-08;b = 8.020341e-04;
		break;
	case 14:
		a = 8.496906e-08;b = 1.305099e-03;
		break;
	case 15:
		a = 7.695089e-08;b = 1.467401e-03;
		break;
	case 16:
		a = 7.259249e-08;b = 1.567108e-03;
		break;
	case 17:
		a = 5.502464e-08;b = 1.924499e-03;
		break;
	case 18:
		a = 4.896508e-08;b = 2.127603e-03;
		break;
	case 19:
		a = -4.894562e-08;b = 2.539403e-03;
		break;
	case 20:
		a = 9.298994e-08;b = 1.322091e-03;
		break;
	case 21:
		a = 9.523190e-08;b = 1.372547e-03;
		break;
	case 22:
		a = 1.064183e-07;b = 1.156072e-03;
		break;
	case 23:
		a = 1.249491e-07;b = 8.652940e-04;
		break;

	default:
		return;
	}
	double rpm2_1 = (-b+sqrt(b*b+4*a*force))/(2*a);
	double rpm2_2 = (-b-sqrt(b*b+4*a*force))/(2*a);
	if (rpm2_1>0)
	{
		rpm2 = rpm2_1;
	}else
	{
		rpm2 = rpm2_2;
	}

	output2 = (46.1221+sqrt(46.1221*46.1221-4*0.0235*(2.4613e04-rpm2)))/(2*0.0235);
	if(sin_roll>=0)
	{
		output[3] = output2;
	}else
	{
		output[2] = output2;
	}


}
