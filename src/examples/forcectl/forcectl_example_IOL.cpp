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

#include "forcectl_example.hpp"

px4::AppState Forcectl::appState;

typedef struct
{
	uint64_t last_timestamp;
	float a0, a1, a2, a3;
	float c0, c1, c2, c3;
	float d0, d1, d2, d3;
    float k0;
    float err;
    float motorSpeed;
    float desiredThrust, last_desiredThrust;
    

}input_output_linearization_controller;

void Input_Output_Linearization(IOL_Controller *IOLC)
{
	if(IOLC == NULL)
    {
		return;
	}

	uint64_t time_now = hrt_absolute_time();
	float dt = math::constrain(((time_now - IOLC->last_timestamp) * 1e-6f), 0.001f, 0.05f);

    motorSpeed_2 = IOLC->motorSpeed * IOLC->motorSpeed;
    dot_desiredThrust = (IOLC->desiredThrust - IOLC->last_desiredThrust) / dt;
	v = IOLC->k0 * IOLC->err;
	u = (v - (4 * IOLC->c3 * motorSpeed_2 * IOLC->motorSpeed + 3 * IOLC->c2 * motorSpeed_2 + 2 * IOLC->c1 * IOLC->motorSpeed + IOLC->c0)
        * (IOLC->a0 * IOLC->motorSpeed + IOLC->a1 * motorSpeed_2 + IOLC->a2 * motorSpeed_2 * IOLC->motorSpeed + IOLC->a3 * motorSpeed_2 * motorSpeed_2)
        / (IOLC->b0 * (4 * IOLC->c3 * motorSpeed_2 * IOLC->motorSpeed + 3 * IOLC->c2 * motorSpeed_2 + 2 * IOLC->c1 * IOLC->motorSpeed + IOLC->c0)));
    u = math::constrain(u, 0.0f, 1.0f);
    u_2 = u * u;
    IOLC->motorSpeed = IOLC->d3 * u_2 * u_2 + IOLC->d2 * u_2 * u + IOLC->d1 * u_2 + IOLC->d0 * u;

    IOLC->last_desiredThrust = IOLC->desiredThrust;
	IOLC->last_timestamp = time_now;
}

int Forcectl::main()
{
	appState.setRunning(true);

	/* subscribe to adc_report topic */
	int adcdata_sub_fd = orb_subscribe(ORB_ID(adc_report));
	/* limit the update rate to 200 Hz */
	orb_set_interval(adcdata_sub_fd, 5);

	/* subscribe to actuator_controls_3 topic */
	int forceexp_sub_rc_fd = orb_subscribe(ORB_ID(actuator_controls_3));
	/* limit the update rate to 50 Hz */
	orb_set_interval(forceexp_sub_rc_fd, 20);

	/* subscribe to actuator_controls_0 topic */
	int forceexp_sub_fc_fd = orb_subscribe(ORB_ID(actuator_controls_0));
	/* limit the update rate to 100 Hz */
	orb_set_interval(forceexp_sub_fc_fd, 10);

	/* subscribe to rc_channels topic */
	int rc_sub_fd = orb_subscribe(ORB_ID(rc_channels));
	/* limit the update rate to 50 Hz */
	orb_set_interval(rc_sub_fd, 20);

	/* subscribe to vehicle_attitude topic */
	int atti_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	/* limit the update rate to 50 Hz */
	orb_set_interval(atti_sub_fd, 20);

	/* advertise forcectl_forcedata topic */
	memset(&force_data, 0, sizeof(force_data));
	orb_advert_t forcedata_pub = orb_advertise(ORB_ID(forcectl_forcedata), &force_data);

	/* advertise forcectl_controldata topic */
	memset(&control_data, 0, sizeof(control_data));
	orb_advert_t controldata_pub = orb_advertise(ORB_ID(forcectl_controldata), &control_data);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = adcdata_sub_fd,   .events = POLLIN },
		{ .fd = forceexp_sub_rc_fd,   .events = POLLIN },
		{ .fd = forceexp_sub_fc_fd,   .events = POLLIN },
	};

	static incremental_PID incre_pid{};
	static positional_PID posi_pid{};
	static ADRC adrc{};
	float _battery_status_scale{0.0f};

	_forcectl_lowpass_filter.reset(0.0);
	_forcectl_torque_lowpass_filter.reset(0.0);
	_incremental_pid_pout_lowpass_filter.reset(0.0);
	_incremental_pid_dout_lowpass_filter.reset(0.0);
	_positional_pid_pout_lowpass_filter.reset(0.0);
	_positional_pid_dout_lowpass_filter.reset(0.0);

	while(appState.isRunning()){
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 3, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret > 0) {

			if (fds[0].revents & POLLIN) {
				orb_copy(ORB_ID(adc_report), adcdata_sub_fd, &adc);
				force_data.timestamp = adc.timestamp;
				static uint64_t forcedata_last_timestamp = force_data.timestamp;
				force_data.force_max = _param_forcectl_force_max.get();
				force_data.force_raw_data[0] = force_data.force_max*(2048 - adc.raw_data[4])/2048.0f;
				force_data.force_lowpass_filtered_data[0] = _forcectl_lowpass_filter.apply(force_data.force_raw_data[0]);
				/* force_data.torque_max = 0.3f * force_data.force_max;
				force_data.torque_raw_data = force_data.torque_max*(2048 - adc.raw_data[10])/2048.0f;
				force_data.torque_lowpass_filtered_data = _forcectl_torque_lowpass_filter.apply(force_data.torque_raw_data); */

				float dt = (float)(force_data.timestamp - forcedata_last_timestamp)/1000000.0f;
				force_data.force_kf_filtered_data[0] = forcectl_kf_filter.force_kf_filter(dt, force_data.force_raw_data[0]);
				//force_data.torque_kf_filtered_data = forcectl_torque_kf_filter.force_kf_filter(dt, force_data.torque_raw_data);
				forcedata_last_timestamp = force_data.timestamp;
				orb_publish(ORB_ID(forcectl_forcedata), forcedata_pub, &force_data);
			}

			if (fds[1].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_3), forceexp_sub_rc_fd, &force_exp_from_rc);
			}

			if (fds[2].revents & POLLIN) {
				orb_copy(ORB_ID(actuator_controls_0), forceexp_sub_fc_fd, &force_exp_from_fc);
			}
		}

		parameters_update();
		orb_copy(ORB_ID(vehicle_attitude), atti_sub_fd, &vehicle_attitude);
		if(_param_forcectl_force_exp.get())
		{
			control_data.weight_com = _param_forcectl_weight.get();
			if(_param_forcectl_weight_compensation.get())
			{
				float pitch = asin(-2.0f * vehicle_attitude.q[1] * vehicle_attitude.q[3] + 2.0f * vehicle_attitude.q[0] * vehicle_attitude.q[2]);
				pitch = (pitch > 3.14159f/2) ? 3.14159f/2 : ((pitch < -3.14159f/2) ? -3.14159f/2 : pitch);
				control_data.weight_com = control_data.weight_com * (float)cos(pitch);
			}
			control_data.force_exp = (_param_forcectl_angacc_to_force.get() * force_exp_from_fc.control[1]) + control_data.weight_com;
			control_data.force_exp = (control_data.force_exp > force_data.force_max) ? force_data.force_max : ((control_data.force_exp < 0.0f) ? 0.0f : control_data.force_exp);
		}
		else
		{
			control_data.force_exp = force_data.force_max * force_exp_from_rc.control[3];
			control_data.force_exp = (control_data.force_exp > force_data.force_max) ? force_data.force_max : ((control_data.force_exp < 0.0f) ? 0.0f : control_data.force_exp);
		}

		orb_copy(ORB_ID(rc_channels), rc_sub_fd, &rc_channals_data);
		if((rc_channals_data.channels[4] < 0.0f)||(rc_channals_data.channels[5] < 0.3f))//||(rc_channals_data.channels[5] > 0.3f))
		{
			memset(&incre_pid, 0, sizeof(incre_pid));
			memset(&posi_pid, 0, sizeof(posi_pid));
		}
		if((rc_channals_data.channels[4] < 0.0f)||(rc_channals_data.channels[5] < 0.3f))
		{
			memset(&adrc, 0, sizeof(adrc));
		}

		if(rc_channals_data.channels[5] > -0.3f)//&&(rc_channals_data.channels[5] < 0.3f))
		{
			control_data.kp = _param_forcectl_pid_p.get();
			control_data.ki = _param_forcectl_pid_i.get();
			control_data.kd = _param_forcectl_pid_d.get();
			control_data.force_error = control_data.force_exp - force_data.force_kf_filtered_data[0];

			if(_param_forcectl_pid_mode.get() == 0)
			{
				incre_pid.Kp = control_data.kp;
				incre_pid.Ki = control_data.ki;
				incre_pid.limit_i = _param_forcectl_pid_limit_i.get();
				incre_pid.Kd = control_data.kd;
				incre_pid.err = control_data.force_error;

				Incremental_PID_calculate(&incre_pid);
				control_data.i_output = incre_pid.i_out;

				control_data.force_control_out = incre_pid.output;
			}
			else if(_param_forcectl_pid_mode.get() == 1)
			{
				posi_pid.Kp = control_data.kp;
				posi_pid.Ki = control_data.ki;
				posi_pid.limit_i = _param_forcectl_pid_limit_i.get();
				posi_pid.factorbase_i = _param_forcectl_pid_factorcase_i.get();
				posi_pid.Kd = control_data.kd;
				posi_pid.err = control_data.force_error;

				Positional_PID_calculate(&posi_pid);
				control_data.i_output = posi_pid.i_out;

				control_data.force_control_out = posi_pid.output;
			}
		}
		else if(rc_channals_data.channels[5] < -0.3f)
		{
			control_data.force_control_out = 0.0f;
		}

		// scale effort by battery status if enabled
		if (_param_forcectl_battery_compensation.get()) {
			if (_battery_status_sub.updated()) {
				if (_battery_status_sub.copy(&battery_status) && battery_status.connected && battery_status.scale > 0.f) {
					control_data.battery_scale = battery_status.scale;
				}
			}

			if (_battery_status_scale > 0.0f) {
				control_data.force_control_out *= control_data.battery_scale;
			}
		}

		/*Td=(1-alpha)*omiga + aplha*omiga2*/
		float forcectl_alpha = _param_forcectl_output_alpha.get();
		forcectl_alpha = (forcectl_alpha < 0.01f) ? 0.01f : ((forcectl_alpha > 1.0f) ? 1.0f : forcectl_alpha);
		control_data.force_control_out = ((float)sqrt((1.0f - forcectl_alpha)*(1.0f - forcectl_alpha) + 4.0f*forcectl_alpha*control_data.force_control_out) + (forcectl_alpha - 1.0f))/(2.0f * forcectl_alpha);
		control_data.force_control_out = (control_data.force_control_out < 0) ? 0 : ((control_data.force_control_out > 1.0f) ? 1.0f : control_data.force_control_out);

		control_data.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(forcectl_controldata), controldata_pub, &control_data);

		px4_usleep(1000);
	}

	return 0;
}
