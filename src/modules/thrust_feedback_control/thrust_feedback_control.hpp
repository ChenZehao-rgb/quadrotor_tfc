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
#pragma once

#include <stdio.h>                     // 标准输入输出库
#include <termios.h>                   // 终端I/O接口
#include <unistd.h>                    // POSIX 操作系统 API 接口
#include <stdbool.h>                   // 定义 bool 类型
#include <errno.h>                     // 错误码定义
#include <drivers/drv_hrt.h>           // PX4 高精度定时器相关的驱动程序头文件
#include <string.h>                    // 字符串操作函数
#include <systemlib/err.h>             // 系统错误处理库（可能已经被替代，不推荐使用）
#include <nuttx/config.h>              // NuttX 配置
#include <fcntl.h>                     // 文件控制定义
#include <sys/types.h>                 // 定义数据类型，如 `size_t`
#include <sys/stat.h>                  // 文件状态定义
#include <poll.h>                      // 多路复用输入输出

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/init.h>

#include <stdint.h>
#include <math.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/barometric_force_sensor.h>
#include <uORB/topics/thrust_data.h>
#include <uORB/topics/thrust_control_data.h>
#include <uORB/topics/thrust_desired_data.h>
#include <uORB/topics/actuator_controls.h>

#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_status.h>
#include "thrust_kalman_filter.hpp"
#include "FilteredDerivative.hpp"

#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

using namespace time_literals;

class ThrustFeedbackControl : public ModuleBase<ThrustFeedbackControl>, public ModuleParams
{
public:
    ThrustFeedbackControl() : ModuleParams(nullptr) {}
    ~ThrustFeedbackControl() override {}

    int main();

    static px4::AppState appState;

private:

    void	parameters_update();

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::TFC_THRUST_MAX>) _param_tfc_thrust_max,
        (ParamFloat<px4::params::TFC_IOLC_K>) _param_tfc_iolc_k,
        (ParamFloat<px4::params::TFC_IOLC_KI>) _param_tfc_iolc_ki,
        (ParamFloat<px4::params::TFC_LIM_I>) _param_tfc_lim_i,
        (ParamFloat<px4::params::TFC_ALPHA>) _param_tfc_alpha,
        (ParamFloat<px4::params::TFC_IOLC_KP1>) _param_tfc_iolc_kp1,
        (ParamFloat<px4::params::TFC_IOLC_KP2>) _param_tfc_iolc_kp2,
        (ParamFloat<px4::params::TFC_IOLC_KP3>) _param_tfc_iolc_kp3,
        (ParamFloat<px4::params::TFC_IOLC_KP4>) _param_tfc_iolc_kp4,
        (ParamFloat<px4::params::TFC_START>) _param_tfc_start,
        (ParamFloat<px4::params::TFC_IOLC_KFF>) _param_tfc_iolc_kff,
        (ParamFloat<px4::params::TFC_IOLC_KFF_1>) _param_tfc_iolc_kff_1,
        (ParamFloat<px4::params::TFC_IOLC_KFF_2>) _param_tfc_iolc_kff_2,
        (ParamFloat<px4::params::TFC_IOLC_KFF_3>) _param_tfc_iolc_kff_3,
        (ParamFloat<px4::params::TFC_IOLC_KFF_4>) _param_tfc_iolc_kff_4,
        (ParamFloat<px4::params::TFC_PWM_TO_THR_1>) _param_tfc_pwm_to_thrust_factor1,
        (ParamFloat<px4::params::TFC_PWM_TO_THR_2>) _param_tfc_pwm_to_thrust_factor2,
        (ParamFloat<px4::params::TFC_PWM_TO_THR_3>) _param_tfc_pwm_to_thrust_factor3,
        (ParamFloat<px4::params::TFC_PWM_TO_THR_4>) _param_tfc_pwm_to_thrust_factor4,
        (ParamFloat<px4::params::TFC_MS_FF>) _param_tfc_ms_ff,
        (ParamFloat<px4::params::TFC_PID_KP>) _param_tfc_pid_kp,
        (ParamFloat<px4::params::TFC_PID_KI>) _param_tfc_pid_ki,
        (ParamFloat<px4::params::TFC_PID_KD>) _param_tfc_pid_kd,
        (ParamFloat<px4::params::TFC_FAC_I>) _param_tfc_fac_i,
        (ParamFloat<px4::params::TFC_PID_LIM_I>) _param_tfc_pid_lim_i,
        (ParamFloat<px4::params::TFC_IOLC_K1>) _param_tfc_iolc_k1,
        (ParamFloat<px4::params::TFC_IOLC_K2>) _param_tfc_iolc_k2,
        (ParamFloat<px4::params::TFC_IOLC_K3>) _param_tfc_iolc_k3,
        (ParamFloat<px4::params::TFC_IOLC_K4>) _param_tfc_iolc_k4,
        (ParamFloat<px4::params::SENSOR1_BIAS1>) _param_sensor1_bias1,
        (ParamFloat<px4::params::SENSOR1_BIAS2>) _param_sensor1_bias2,
        (ParamFloat<px4::params::SENSOR2_BIAS1>) _param_sensor2_bias1,
        (ParamFloat<px4::params::SENSOR2_BIAS2>) _param_sensor2_bias2,
        (ParamFloat<px4::params::SENSOR3_BIAS1>) _param_sensor3_bias1,
        (ParamFloat<px4::params::SENSOR3_BIAS2>) _param_sensor3_bias2,
        (ParamFloat<px4::params::SENSOR4_BIAS1>) _param_sensor4_bias1,
        (ParamFloat<px4::params::SENSOR4_BIAS2>) _param_sensor4_bias2,
        (ParamFloat<px4::params::THR_USE_FIL>) _param_tfc_use_filtered_thrust,
        (ParamFloat<px4::params::ALPHA_TAU>) _param_alpha_tau
    )

    uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};

    thrust_data_s thrustdata = {};
    uORB::Publication<thrust_data_s>  _thrustdata_pub{ORB_ID(thrust_data)};
    ThrustKalmanFilter thrust_kalman_filter{1.0f};

    struct actuator_controls_s force_exp_from_rc{};


    // 发布控制数据话题
    thrust_control_data_s thrustcontroldata = {};
    uORB::Publication<thrust_control_data_s> _thrustcontroldata_pub{ORB_ID(thrust_control_data)};

    // 三阶拟合参数
    double iolc_a3 = -2.252e-7;
    double iolc_a2 = 9.821e-4;
    double iolc_a1 = -6.925;
    double iolc_b0 = 6.0018e4;

    // 二阶参数
    // double iolc_a3 = 0;
    // double iolc_a2 = -0.1663;
    // double iolc_a1 = -71.6385;
    // double iolc_b0 = 6.0018e4;

    double U_b = 23.54; // 电池标称电压23.54V
    double R_m = 0.055; // 电机电阻0.055欧姆
    double R_e = 0.03; // 电调内阻0.03欧姆
    double C_e = 0.028; // 反电动势常数
    double J_m = 1.292e-4; // 电机转动惯量
    double f_m = 3.217e-5; // 电机粘性摩擦系数
    double alpha = 0.2113;

    // double iolc_c3 = 3.379e-10*9.5493*9.5493*9.5493;
    // double iolc_c2 = -1.518e-6*9.5493*9.5493;
    // double iolc_c1 = 3.573e-3*9.5493;
    double iolc_c3 = 3.379e-10/9.81;
    double iolc_c2 = -1.518e-6/9.81;
    double iolc_c1 = 3.573e-3/9.81;
    float Thrust_Max; // 单轴最大升力为2kg
    double motorSpeed_FF = 50.0;

    float iolc_d1 = 1.755;
    float iolc_d2 = 0.745;

    matrix::Vector<float, 4> _thrust_desired;
    matrix::Vector<float, 4> _thrust_measure;
    matrix::Vector<float, 4> _control_output;
    matrix::Vector<float, 4> _total_output;
    matrix::Vector<float, 4> _iolc_u_ff;
    matrix::Vector<float, 4> _thrust_kalman_filter_control_out;
    matrix::Vector<float, 4> _thrust_desired_dot;
    matrix::Vector<float, 4> _u_fb_coeff;

    struct rc_channels_s rc_channals_data{};

    ThrustDerivative<float> td;
    uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
    vehicle_status_s _vehicle_status{};
    bool _armed{false};
};
