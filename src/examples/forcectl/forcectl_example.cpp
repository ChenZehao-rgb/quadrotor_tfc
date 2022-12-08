/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file forcectl_example.cpp
 * Test application example for force feedback control
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */

#include "forcectl_example.hpp"

px4::AppState Forcectl::appState;
math::LowPassFilter2p<float>	_pid_pout_lowpass_filter{100.f, 30.f};
math::LowPassFilter2p<float>	_pid_dout_lowpass_filter{100.f, 10.f};

typedef struct{
	float Kp;
	float Ki;
	float Kd;
	float p_out;
	float i_out;
	float d_out;
	float err;
	float last_err;
	float previous_err;
	float output;
}incremental_PID;

typedef struct{
	uint64_t last_timestamp;
	float dt;
	float force_exp;
	float force_mea;
	float TD_v1;
	float TD_v2;
	float ESO_z1;
	float ESO_z2;
	float ESO_z3;
	float beta1;
	float beta2;
	float beta3;
	float b0;
	float Kp;
	float Kd;
	float output;
}ADRC;

void Incremental_PID_calculate(incremental_PID *pid)
{
	if(pid == NULL){
		return;
	}

	pid->p_out = pid->Kp * (pid->err - pid->last_err);
	pid->i_out = pid->Ki * pid->err;
	pid->d_out = pid->Kd * (pid->err - 2.0f*pid->last_err + pid->previous_err);

	pid->p_out = _pid_pout_lowpass_filter.apply(pid->p_out);
	pid->d_out = _pid_dout_lowpass_filter.apply(pid->d_out);

	pid->output += pid->p_out + pid->i_out + pid->d_out;
	pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);

	pid->previous_err = pid->last_err;
	pid->last_err = pid->err;
}

float ADRC_sign(float in1)
{
	return ((in1 > 0.0f) ? 1.0f : ((abs(in1 - 0.0f) < 1e-7) ? 0.0f : -1.0f));
}

float ADRC_fsg(float in1, float in2)
{
	return (ADRC_sign(in1 + in2) - ADRC_sign(in1 - in2))/2.0f;
}

float ADRC_fhan(float in1, float in2, float in3, float in4)
{
	float d = in3 * in4 * in4;
	float a0 = in4 * in2;
	float y = in1 + a0;
	float a1 = sqrt(d*(d+ 8.0f*abs(y)));
	float a2 = a0 + ADRC_sign(y)*(a1-d)/2.0f;
	float a = (a0 + y)*ADRC_fsg(y,d) + a2*(1.0f-ADRC_fsg(y,d));
	float fhan = -in3*(a/d)*ADRC_fsg(a,d) - in3*ADRC_sign(a)*(1.0f - ADRC_fsg(a,d));

	return fhan;
}

float ADRC_fal(float in1, float in2, float in3)
{
	float s = (ADRC_sign(in1 +in3) - ADRC_sign(in1 - in3))/2.0f;
	float fal = (float)(in1*s/((float)pow(in3, (1.0f - in2)))) + (float)((float)pow(abs(in1), in2)*ADRC_sign(in1)*(1.0f - s));

	return fal;
}

void ADRC_calculate(ADRC *adrc)
{
	if(adrc == NULL){
		return;
	}

	uint64_t time_now = hrt_absolute_time();
	float dt = (float)(time_now - adrc->last_timestamp)/1000000.0f;
	dt = dt < 0.001f ? 0.02f : dt;
	adrc->dt = dt;
	adrc->last_timestamp = time_now;

	/*ADRC-TD*/
	float e_TD = adrc->TD_v1 - adrc->force_exp;
	float fh = ADRC_fhan(e_TD, adrc->TD_v2, 400.0f, dt);
	adrc->TD_v1 = adrc->TD_v1 + adrc->TD_v2*dt;
	adrc->TD_v2 = adrc->TD_v2 + fh*dt;

	/*CONTROLLOR*/
	float e1_CTL = adrc->TD_v1 - adrc->ESO_z1;
	float e2_CTL = adrc->TD_v2 - adrc->ESO_z2;
	float u0_CTL = adrc->Kp*e1_CTL + adrc->Kd*e2_CTL;
	float u_CTL = u0_CTL - adrc->ESO_z3/adrc->b0;

	/*ESO*/
	float e_ESO = adrc->ESO_z1 - adrc->force_mea;
	float fe = ADRC_fal(e_ESO, 0.5f, dt);
	float fe1 = ADRC_fal(e_ESO, 0.25f, dt);
	adrc->ESO_z1 = adrc->ESO_z1 + dt*(adrc->ESO_z2 - adrc->beta1*e_ESO);
	adrc->ESO_z2 = adrc->ESO_z2 + dt*(adrc->ESO_z3 - adrc->beta2*fe + adrc->b0*u_CTL);
	adrc->ESO_z3 = adrc->ESO_z3 + dt*(-adrc->beta3*fe1);

	/*OUTPUT*/
	adrc->output = (u_CTL > 1.0f) ? 1.0f : ((u_CTL < 0.0f) ? 0.0f : u_CTL);
}

int Forcectl::main()
{
	appState.setRunning(true);

	/* subscribe to forcectl_forcedata topic */
	int forcedata_sub_fd = orb_subscribe(ORB_ID(forcectl_forcedata));
	/* limit the update rate to 50 Hz */
	orb_set_interval(forcedata_sub_fd, 20);

	/* subscribe to actuator_controls_3 topic */
	int forceexp_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3));
	/* limit the update rate to 50 Hz */
	orb_set_interval(forceexp_sub_fd, 20);

	/* subscribe to rc_channels topic */
	int pid_sub_fd = orb_subscribe(ORB_ID(rc_channels));
	/* limit the update rate to 1 Hz */
	orb_set_interval(pid_sub_fd, 20);

	/* advertise forcectl_controldata topic */
	struct forcectl_controldata_s control_data;
	memset(&control_data, 0, sizeof(control_data));
	orb_advert_t controldata_pub = orb_advertise(ORB_ID(forcectl_controldata), &control_data);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = forcedata_sub_fd,   .events = POLLIN },
		{ .fd = forceexp_sub_fd,   .events = POLLIN },
	};

	struct forcectl_forcedata_s forcedata = {};
	struct actuator_controls_s force_exp_from_rc = {};
	struct rc_channels_s rc_channals_data = {};
	static incremental_PID incre_pid = {};
	static ADRC adrc = {};

	_pid_pout_lowpass_filter.reset(0.0);
	_pid_dout_lowpass_filter.reset(0.0);

	while(appState.isRunning()){
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 2, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret > 0) {

			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(forcectl_forcedata), forcedata_sub_fd, &forcedata);
			}

			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_3), forceexp_sub_fd, &force_exp_from_rc);
				control_data.force_exp = forcedata.force_max*force_exp_from_rc.control[3];
			}

		}

		orb_copy(ORB_ID(rc_channels), pid_sub_fd, &rc_channals_data);
		if((rc_channals_data.channels[4] < 0.0f)||(rc_channals_data.channels[5] < -0.3f)||(rc_channals_data.channels[5] > 0.3f))
		{
			memset(&incre_pid, 0, sizeof(incre_pid));
		}
		/* if((rc_channals_data.channels[4] < 0.0f)||(rc_channals_data.channels[5] < 0.3f))
		{
			memset(&adrc, 0, sizeof(adrc));
		}
 */
		if((rc_channals_data.channels[5] > -0.3f)&&(rc_channals_data.channels[5] < 0.3f))
		{
			control_data.kp = 2.0f*(rc_channals_data.channels[6] + 1.0f)/2.0f;
			control_data.ki = 0.2f*(rc_channals_data.channels[7] + 1.0f)/2.0f;
			control_data.kd = 5.0f*(rc_channals_data.channels[8] + 1.0f)/2.0f;

			control_data.force_error = control_data.force_exp - forcedata.force_kf_filtered_data;
			incre_pid.Kp = control_data.kp;
			incre_pid.Ki = control_data.ki;
			incre_pid.Kd = control_data.kd;
			incre_pid.err = control_data.force_error;

			Incremental_PID_calculate(&incre_pid);

			control_data.force_control_out = incre_pid.output;
		//}
		//else if(rc_channals_data.channels[5] > 0.3f)
		//{
			adrc.b0 = (float)0.028e7/(1.292f*0.055f);
			adrc.beta1 = (float)1e4;
			adrc.beta2 = (float)1e7;
			adrc.beta3 = (float)1e10;
			adrc.Kp = 0.08f;
			adrc.Kd = 0.005f;
			adrc.force_exp = control_data.force_exp;
			adrc.force_mea = forcedata.force_raw_data;

			ADRC_calculate(&adrc);

			//control_data.force_control_out = adrc.output;
		}
		else if(rc_channals_data.channels[5] < -0.3f)
		{
			control_data.force_control_out = 0.0f;
		}

#if SAVE_FORCECTL_ADRC_DATA
		forcectl_ADRC_data.dt = adrc.dt;
		forcectl_ADRC_data.td_v1 = adrc.TD_v1;
		forcectl_ADRC_data.td_v2 = adrc.TD_v2;
		forcectl_ADRC_data.eso_z1 = adrc.ESO_z1;
		forcectl_ADRC_data.eso_z2 = adrc.ESO_z2;
		forcectl_ADRC_data.eso_z3 = adrc.ESO_z3;
		forcectl_ADRC_data.beta1 = adrc.beta1;
		forcectl_ADRC_data.beta2 = adrc.beta2;
		forcectl_ADRC_data.beta3 = adrc.beta3;
		forcectl_ADRC_data.b0 = adrc.b0;
		forcectl_ADRC_data.kp = adrc.Kp;
		forcectl_ADRC_data.kd = adrc.Kd;
		forcectl_ADRC_data.output = adrc.output;

		forcectl_ADRC_data.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(forcectl_adrc_data), adrc_data_pub, &forcectl_ADRC_data);
#endif

		control_data.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(forcectl_controldata), controldata_pub, &control_data);

		px4_usleep(10000);
	}

	return 0;
}
