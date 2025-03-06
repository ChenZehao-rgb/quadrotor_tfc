/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file frocectl_example.hpp
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */
#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/init.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <sched.h>
#include <uORB/uORB.h>
#include <uORB/topics/forcectl_forcedata.h>
#include <uORB/topics/forcectl_controldata.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/forcectl_adrc_data.h>
#include <uORB/topics/battery_status.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <examples/forcectl/forcectl_kf_filter.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/barometric_force_sensor.h>
#include <uORB/topics/thrust_iolc_ekf.h>

using namespace time_literals;

#define SAVE_FORCECTL_ADRC_DATA 0

class Forcectl : public ModuleBase<Forcectl>, public ModuleParams
{
public:
	Forcectl() : ModuleParams(nullptr) {}

	~Forcectl() override {}

	int main();

	static px4::AppState appState; /* track requests to terminate app */

private:
	/**
	 * initialize some vectors/matrices from parameters
	 */
	void	parameters_update();

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FORCECTL_FOR_MAX>) _param_forcectl_force_max,
		(ParamInt<px4::params::FORCECTL_FOR_EXP>) _param_forcectl_force_exp,
		(ParamFloat<px4::params::FORCECTL_AA2F_K>) _param_forcectl_angacc_to_force,
		(ParamFloat<px4::params::FORCECTL_WEIGHT>) _param_forcectl_weight,
		(ParamInt<px4::params::FORCECTL_WEI_COM>) _param_forcectl_weight_compensation,
		(ParamFloat<px4::params::FORCECTL_ALPHA>) _param_forcectl_output_alpha,
		(ParamInt<px4::params::FORCECTL_BAT_COM>) _param_forcectl_battery_compensation,
		(ParamInt<px4::params::FORCECTL_PID_MOD>) _param_forcectl_pid_mode,
		(ParamFloat<px4::params::FORCECTL_PID_P>) _param_forcectl_pid_p,
		(ParamFloat<px4::params::FORCECTL_PID_I>) _param_forcectl_pid_i,
		(ParamFloat<px4::params::FORCECTL_LIM_I>) _param_forcectl_pid_limit_i,
		(ParamFloat<px4::params::FORCECTL_FAC_I>) _param_forcectl_pid_factorcase_i,
		(ParamFloat<px4::params::FORCECTL_PID_D>) _param_forcectl_pid_d,
		(ParamFloat<px4::params::FORCECTL_ADRC_B0>) _param_forcectl_adrc_b0,
		(ParamFloat<px4::params::FORCECTL_ADRC_B1>) _param_forcectl_adrc_b1,
		(ParamFloat<px4::params::FORCECTL_ADRC_B2>) _param_forcectl_adrc_b2,
		(ParamFloat<px4::params::FORCECTL_ADRC_B3>) _param_forcectl_adrc_b3,
		(ParamFloat<px4::params::FORCECTL_ADRC_KP>) _param_forcectl_adrc_kp,
		(ParamFloat<px4::params::FORCECTL_ADRC_KD>) _param_forcectl_adrc_kd,
        (ParamFloat<px4::params::FORCECTL_IOLC_K0>) _param_forcectl_iolc_k0,
        (ParamFloat<px4::params::FORCECTL_IOLC_Q>) _param_forcectl_iolc_q,
        (ParamFloat<px4::params::FORCECTL_IOLC_R>) _param_forcectl_iolc_r,
		(ParamFloat<px4::params::FORCECTL_IOLC_D>) _param_forcectl_iolc_d
	)

	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription 		_battery_status_sub{ORB_ID(battery_status)};

	ForcectlKfFilter forcectl_kf_filter{1.0f};
	ForcectlKfFilter forcectl_torque_kf_filter{1.0f};

    struct thrust_iolc_ekf_s iolc_ekf_data;
    struct barometric_force_sensor_s sensordata;
	struct forcectl_forcedata_s force_data{};
	struct forcectl_controldata_s control_data{};
	struct adc_report_s adc{};
	struct actuator_controls_s force_exp_from_rc{};
	struct actuator_controls_s force_exp_from_fc{};
	struct rc_channels_s rc_channals_data{};
	struct vehicle_attitude_s vehicle_attitude{};
	struct battery_status_s battery_status{};

    // double iolc_ekf_a3 = 1.034e-7;
    // double iolc_ekf_a2 = -1.285e-3;
    // double iolc_ekf_a1 = 9.251;
    // double iolc_ekf_b0 = -3.008e4;
	double iolc_ekf_a3 = 0;
    double iolc_ekf_a2 = -0.1663;
    double iolc_ekf_a1 = -71.6385;
    double iolc_ekf_b0 = 6.0018e4;
    double iolc_ekf_c3 = 3.379e-10*9.5493*9.5493*9.5493;
    double iolc_ekf_c2 = -1.518e-6*9.5493*9.5493;
    double iolc_ekf_c1 = 3.573e-3*9.5493;

#if SAVE_FORCECTL_ADRC_DATA
	forcectl_adrc_data_s forcectl_ADRC_data = {0};
	orb_advert_t adrc_data_pub = orb_advertise(ORB_ID(forcectl_adrc_data), &forcectl_ADRC_data);
#endif
};