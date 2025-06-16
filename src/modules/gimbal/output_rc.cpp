/****************************************************************************
*
*   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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


#include "output_rc.h"

#include <uORB/topics/actuator_controls.h>
#include <px4_platform_common/defines.h>
#include <matrix/matrix/math.hpp>

using math::constrain;

namespace gimbal
{

OutputRC::OutputRC(const Parameters &parameters)
	: OutputBase(parameters)
{
}

void OutputRC::update(const ControlData &control_data, bool new_setpoints)
{
	if (new_setpoints) {
		_retract_gimbal = control_data.gimbal_shutter_retract;
		_set_angle_setpoints(control_data);
	}

	_handle_position_update(control_data);

	hrt_abstime t = hrt_absolute_time();
	_calculate_angle_output(t);

	_stream_device_attitude_status();

	// _angle_outputs are in radians, actuator_controls are in [-1, 1]
	actuator_controls_s actuator_controls{};
	actuator_controls.timestamp = hrt_absolute_time();

	_forcectl_controldata_sub.update(&forcectl_control_data);
	_rc_channels_sub.update(&rc_channals_data);
	_vehicle_status_sub.update(&vehicle_status);
	_actuator_controls_sub.update(&actuator_controls_data);
	_thrust_control_data_sub.update(&thrustcontroldata);

	// /*Td=(1-alpha)*omiga + aplha*omiga2*/
	float forcectl_alpha = (rc_channals_data.channels[9] + 1.0f)/2.0f;
	forcectl_alpha = (forcectl_alpha < 0.01f) ? 0.01f : ((forcectl_alpha > 1.0f) ? 1.0f : forcectl_alpha);
	float forcectl_manual_control = ((float)sqrt((1.0f - forcectl_alpha)*(1.0f - forcectl_alpha) + 4.0f*forcectl_alpha*rc_channals_data.channels[2]) + (forcectl_alpha - 1.0f))/(2.0f * forcectl_alpha);
	forcectl_manual_control = (forcectl_manual_control < 0) ? 0 : ((forcectl_manual_control > 1.0f) ? 1.0f : forcectl_manual_control);

	/* 单轴力反馈 */
	// actuator_controls.control[0] = -1.0f;

	// if(!vehicle_status.rc_signal_lost){
	// 	if(rc_channals_data.channels[4] < 0.0f)
	// 	{
	// 		actuator_controls.control[0] = -1.0f;
	// 	}
	// 	else if(rc_channals_data.channels[4] > 0.0f)
	// 	{
	// 		if(rc_channals_data.channels[5] < -0.3f)
	// 		{
	// 			actuator_controls.control[0] = forcectl_manual_control*2.0f-1.0f;
	// 		}
	// 		else if(rc_channals_data.channels[5] > -0.3f)
	// 		{
	// 			actuator_controls.control[0] = forcectl_control_data.force_control_out*2.0f-1.0f;
	// 		}
	// 	}
	// }

	/* 整机力反馈 */
	actuator_controls.control[0] = -1.0f;
	actuator_controls.control[1] = -1.0f;
	actuator_controls.control[2] = -1.0f;
	actuator_controls.control[3] = -1.0f;

	if(!vehicle_status.rc_signal_lost)
	{
		if(rc_channals_data.channels[4] < 0.0f)
		{
			actuator_controls.control[0] = -1.0f;
			actuator_controls.control[1] = -1.0f;
			actuator_controls.control[2] = -1.0f;
			actuator_controls.control[3] = -1.0f;
		}
		else if(rc_channals_data.channels[4] > 0.0f)
		{
			if(rc_channals_data.channels[5] < -0.3f)
			{
				actuator_controls.control[0] = forcectl_manual_control*2.0f-1.0f;
				actuator_controls.control[1] = forcectl_manual_control*2.0f-1.0f;
				actuator_controls.control[2] = forcectl_manual_control*2.0f-1.0f;
				actuator_controls.control[3] = forcectl_manual_control*2.0f-1.0f;
				escinputpwm.manual_control_1 = actuator_controls.control[0];
				escinputpwm.manual_control_2 = actuator_controls.control[1];
				escinputpwm.manual_control_3 = actuator_controls.control[2];
				escinputpwm.manual_control_4 = actuator_controls.control[3];
			}
			else if(rc_channals_data.channels[5] > -0.3f)
			{
				actuator_controls.control[0] = thrustcontroldata.thrust_control_out1*2.0f-1.0f;
				actuator_controls.control[1] = thrustcontroldata.thrust_control_out2*2.0f-1.0f;
				actuator_controls.control[2] = thrustcontroldata.thrust_control_out3*2.0f-1.0f;
				actuator_controls.control[3] = thrustcontroldata.thrust_control_out4*2.0f-1.0f;
				// actuator_controls.control[0] = -1.0f;
				// actuator_controls.control[1] = -1.0f;
				// actuator_controls.control[2] = -1.0f;
				// actuator_controls.control[3] = -0.8f;
				escinputpwm.thrust_control_1 = actuator_controls.control[0];
				escinputpwm.thrust_control_2 = actuator_controls.control[1];
				escinputpwm.thrust_control_3 = actuator_controls.control[2];
				escinputpwm.thrust_control_4 = actuator_controls.control[3];
			}
		}
	}

	// actuator_controls.control[4] = -0.8f;
	// actuator_controls.control[5] = -0.8f;
	// actuator_controls.control[6] = -0.8f;
	// actuator_controls.control[7] = -0.8f;

	escinputpwm.timestamp = hrt_absolute_time();
	_escinputpwm_pub.publish(escinputpwm);

	_actuator_controls_pub.publish(actuator_controls);

	_last_update = t;
}

void OutputRC::print_status() const
{
	PX4_INFO("Output: AUX");
}

void OutputRC::_stream_device_attitude_status()
{
	gimbal_device_attitude_status_s attitude_status{};
	attitude_status.timestamp = hrt_absolute_time();
	attitude_status.target_system = 0;
	attitude_status.target_component = 0;
	attitude_status.device_flags = gimbal_device_attitude_status_s::DEVICE_FLAGS_NEUTRAL |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_ROLL_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_PITCH_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK;

	matrix::Eulerf euler(_angle_outputs[0], _angle_outputs[1], _angle_outputs[2]);
	matrix::Quatf q(euler);
	q.copyTo(attitude_status.q);

	attitude_status.failure_flags = 0;
	_attitude_status_pub.publish(attitude_status);
}

} /* namespace gimbal */
