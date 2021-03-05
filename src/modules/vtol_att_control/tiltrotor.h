/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

/**
* @file tiltrotor.h
*
* @author Roman Bapst 		<bapstroman@gmail.com>
*
*/

#ifndef TILTROTOR_H
#define TILTROTOR_H
#include "vtol_type.h"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <matrix/Euler.hpp>
#include <matrix/Dcm.hpp>
#include <matrix/Quaternion.hpp>

class Tiltrotor : public VtolType
{

public:

	Tiltrotor(VtolAttitudeControl *_att_controller);
	~Tiltrotor() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void fill_actuator_outputs() override;
	void update_mc_state() override;
	void update_fw_state() override;
	void waiting_on_tecs() override;
	float thrust_compensation_for_tilt();

private:

	struct {
		float tilt_mc;					/**< actuator value corresponding to mc tilt */
		float tilt_transition;			/**< actuator value corresponding to transition tilt (e.g 45 degrees) */
		float tilt_fw;					/**< actuator value corresponding to fw tilt */
		float front_trans_dur_p2;
		float tilt_err_1;                      /*front right servo position compensation*/
		float tilt_err_2;                      /*front left servo position compensation*/
		float tilt_err_3;                      /*back right servo position compensation*/
		float tilt_err_4;                      /*back left servo position compensation*/
                int auto_outputs;
	} _params_tiltrotor;

	struct {
		param_t tilt_mc;
		param_t tilt_transition;
		param_t tilt_fw;
		param_t front_trans_dur_p2;
		param_t tilt_err_1;
		param_t tilt_err_2;
		param_t tilt_err_3;
		param_t tilt_err_4;
		param_t auto_outputs;
	} _params_handles_tiltrotor;

	enum class vtol_mode {
		MC_MODE = 0,			/**< vtol is in multicopter mode */
		TRANSITION_FRONT_P1,	/**< vtol is in front transition part 1 mode */
		TRANSITION_FRONT_P2,	/**< vtol is in front transition part 2 mode */
		TRANSITION_BACK,		/**< vtol is in back transition mode */
		FW_MODE					/**< vtol is in fixed wing mode */
	};

	/**
	 * Specific to tiltrotor with vertical aligned rear engine/s.
	 * These engines need to be shut down in fw mode. During the back-transition
	 * they need to idle otherwise they need too much time to spin up for mc mode.
	 */


	struct {
		vtol_mode flight_mode;			/**< vtol flight mode, defined by enum vtol_mode */
		hrt_abstime transition_start;	/**< absoulte time at which front transition started */
	} _vtol_schedule;

        float _tilt_control{0.0f};
	float _tilt_control1{0.0f};
        float _tilt_control2{0.0f};
	float _tilt_control3{0.0f};
	float _tilt_control4{0.0f};


	hrt_abstime start_time= 0;
	hrt_abstime last_time = 0;
	hrt_abstime takeoff_time= 0;
	hrt_abstime fly_time= 0;
	hrt_abstime fly_end_time= 0;
	hrt_abstime land_time= 0;
	bool start_flag = 1;
	bool takeoff_flag = 0;
	bool water_takeoff = true;
	bool can_not_trun_front_more = false;

	void parameters_update() override;
	void gen_virutal_mc_controls();

};
#endif
