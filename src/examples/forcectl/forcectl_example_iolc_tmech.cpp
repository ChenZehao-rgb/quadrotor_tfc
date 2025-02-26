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

#include "forcectl_example_iolc_tmech.hpp"

 px4::AppState Forcectl::appState;
 math::LowPassFilter2p<float>	_forcectl_lowpass_filter{100.f, 30.f};
 math::LowPassFilter2p<float>	_forcectl_torque_lowpass_filter{100.f, 30.f};
 math::LowPassFilter2p<float>	_incremental_pid_pout_lowpass_filter{100.f, 30.f};
 math::LowPassFilter2p<float>	_incremental_pid_dout_lowpass_filter{100.f, 10.f};
 math::LowPassFilter2p<float>	_positional_pid_pout_lowpass_filter{100.f, 30.f};
 math::LowPassFilter2p<float>	_positional_pid_dout_lowpass_filter{100.f, 10.f};
 
 typedef struct{
	 float Kp;
	 float Ki;
	 float Kd;
	 float p_out;
	 float i_out;
	 float limit_i;
	 float d_out;
	 float err;
	 float last_err;
	 float previous_err;
	 float output;
 }incremental_PID;
 
 typedef struct{
	 uint64_t last_timestamp;
	 float Kp;
	 float Ki;
	 float Kd;
	 float p_out;
	 float i_out;
	 float limit_i;
	 float factorbase_i;
	 float d_out;
	 float err;
	 float last_err;
	 float output;
 }positional_PID;
 
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
	 float u0_CTL;
	 float output;
 }ADRC;
 
 void Forcectl::parameters_update()
 {
	 // Check if parameters have changed
	 if (_parameter_update_sub.updated()) {
		 // clear update
		 parameter_update_s param_update;
		 _parameter_update_sub.copy(&param_update);
 
		 updateParams();
	 }
 }
 
 void Incremental_PID_calculate(incremental_PID *pid)
 {
	 if(pid == NULL){
		 return;
	 }
 
	 pid->p_out = pid->Kp * (pid->err - pid->last_err);
	 pid->i_out = pid->Ki * pid->err;
	 pid->d_out = pid->Kd * (pid->err - 2.0f*pid->last_err + pid->previous_err);
 
	 pid->i_out = math::constrain(pid->i_out, -pid->limit_i, pid->limit_i);
 
	 pid->p_out = _incremental_pid_pout_lowpass_filter.apply(pid->p_out);
	 pid->d_out = _incremental_pid_dout_lowpass_filter.apply(pid->d_out);
 
	 pid->output += pid->p_out + pid->i_out + pid->d_out;
	 pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);
 
	 pid->previous_err = pid->last_err;
	 pid->last_err = pid->err;
 }
 
 void Positional_PID_calculate(positional_PID *pid)
 {
	 if(pid == NULL){
		 return;
	 }
 
	 uint64_t time_now = hrt_absolute_time();
	 float dt = math::constrain(((time_now - pid->last_timestamp) * 1e-6f), 0.001f, 0.05f);
 
	 pid->p_out = pid->Kp * pid->err;
	 pid->d_out = pid->Kd * (pid->err - pid->last_err)/dt;
 
	 float i_factor = pid->err / pid->factorbase_i;
	 i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
	 pid->i_out = pid->i_out + i_factor * pid->Ki * pid->err * dt;
	 pid->i_out = math::constrain(pid->i_out, -pid->limit_i, pid->limit_i);
 
	 pid->p_out = _positional_pid_pout_lowpass_filter.apply(pid->p_out);
	 pid->d_out = _positional_pid_dout_lowpass_filter.apply(pid->d_out);
 
	 pid->output = pid->p_out + pid->i_out + pid->d_out;
	 pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);
 
	 pid->last_err = pid->err;
	 pid->last_timestamp = time_now;
 }
 
 float ADRC_sign(float in1)
 {
	 return ((in1 > 0.0f) ? 1.0f : ((fabs(in1 - 0.0f) < 1e-7) ? 0.0f : -1.0f));
 }
 
 float ADRC_fsg(float in1, float in2)
 {
	 return (ADRC_sign(in1 + in2) - ADRC_sign(in1 - in2))/2.0f;
 }
 
 float ADRC_fhan(double in1, double in2, double in3, double in4)
 {
	 double d = in3 * in4 * in4;
	 double a0 = in4 * in2;
	 double y = in1 + a0;
	 double a1 = sqrt(d*(d + 8.0*fabs(y)));
	 double a2 = a0 + (double)ADRC_sign(y)*(a1 - d)/2.0;
	 double a = (a0 + y)*(double)ADRC_fsg(y,d) + a2*(1.0 - (double)ADRC_fsg(y,d));
	 double fhan = -in3*(a/d)*(double)ADRC_fsg(a,d) - in3*(double)ADRC_sign(a)*(1.0 - (double)ADRC_fsg(a,d));
 
	 return (float)fhan;
 }
 
 float ADRC_fal(float in1, float in2, float in3)
 {
	 float s = (ADRC_sign(in1 + in3) - ADRC_sign(in1 - in3))/2.0f;
	 double fal = (double)in1*(double)s/(pow(in3, (1.0f - in2))) + pow(fabs(in1), in2)*(double)ADRC_sign(in1)*(1.0 - (double)s);
 
	 return (float)fal;
 }
 
 void ADRC_calculate(ADRC *adrc)
 {
	 if(adrc == NULL){
		 return;
	 }
 
	 uint64_t time_now = hrt_absolute_time();
	 float dt = (float)(time_now - adrc->last_timestamp)/1000000.0f;
	 dt = (dt < 0.001f) ? 0.02f : ((dt > 0.04f) ? 0.02f : dt);
	 adrc->dt = dt;
	 adrc->last_timestamp = time_now;
 
	 /*ADRC-TD*/
	 float e_TD = adrc->TD_v1 - adrc->force_exp;
	 float fh = ADRC_fhan(e_TD, adrc->TD_v2, 400.0f, dt);
	 adrc->TD_v1 = adrc->TD_v1 + adrc->TD_v2*dt;
	 adrc->TD_v2 = adrc->TD_v2 + fh*dt;
 
	 /*CONTROLLER*/
	 float e1_CTL = adrc->TD_v1 - adrc->ESO_z1;
	 float e2_CTL = adrc->TD_v2 - adrc->ESO_z2;
	 adrc->u0_CTL += adrc->Kp*e1_CTL + adrc->Kd*e2_CTL;
	 float u_CTL = (adrc->u0_CTL - adrc->ESO_z3)/adrc->b0;
 
	 /*ESO*/
	 float e_ESO = adrc->ESO_z1 - adrc->force_mea;
	 float fe = ADRC_fal(e_ESO, 0.5f, dt);
	 float fe1 = ADRC_fal(e_ESO, 0.25f, dt);
	 adrc->ESO_z1 = adrc->ESO_z1 + dt*(adrc->ESO_z2 - adrc->beta1*e_ESO);
	 adrc->ESO_z2 = adrc->ESO_z2 + dt*(adrc->ESO_z3 - adrc->beta2*fe + adrc->b0*u_CTL);
	 adrc->ESO_z3 = adrc->ESO_z3 + dt*(-adrc->beta3*fe1);
	 // adrc->ESO_z1 = adrc->ESO_z1 + dt*(adrc->ESO_z2 - adrc->beta1*e_ESO + adrc->b0*u_CTL);
	 // adrc->ESO_z2 = adrc->ESO_z2 + dt*(-adrc->beta2*fe);
 
	 /*OUTPUT*/
	 adrc->output = (u_CTL > 1.0f) ? 1.0f : ((u_CTL < 0.0f) ? 0.0f : u_CTL);
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
				 force_data.force_raw_data = force_data.force_max*(2048 - adc.raw_data[4])/2048.0f;
				 force_data.force_lowpass_filtered_data = _forcectl_lowpass_filter.apply(force_data.force_raw_data);
				 /* force_data.torque_max = 0.3f * force_data.force_max;
				 force_data.torque_raw_data = force_data.torque_max*(2048 - adc.raw_data[10])/2048.0f;
				 force_data.torque_lowpass_filtered_data = _forcectl_torque_lowpass_filter.apply(force_data.torque_raw_data); */
 
				 float dt = (float)(force_data.timestamp - forcedata_last_timestamp)/1000000.0f;
				 force_data.force_kf_filtered_data = forcectl_kf_filter.force_kf_filter(dt, force_data.force_raw_data);
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
			 control_data.force_error = control_data.force_exp - force_data.force_kf_filtered_data;
 
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
		 /* else if(rc_channals_data.channels[5] > 0.3f)
		 {
			 adrc.b0 = _param_forcectl_adrc_b0.get();
			 adrc.beta1 = _param_forcectl_adrc_b1.get();
			 adrc.beta2 = _param_forcectl_adrc_b2.get();
			 adrc.beta3 = _param_forcectl_adrc_b3.get();
			 adrc.Kp = _param_forcectl_adrc_kp.get();
			 adrc.Kd = _param_forcectl_adrc_kd.get();
			 adrc.force_exp = control_data.force_exp;
			 adrc.force_mea = force_data.force_kf_filtered_data;
 
			 if(rc_channals_data.channels[4] > 0.0f)
			 {
				 ADRC_calculate(&adrc);
			 }
 
			 control_data.force_control_out = adrc.output;
		 } */
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
 
		 px4_usleep(1000);
	 }
 
	 return 0;
 }