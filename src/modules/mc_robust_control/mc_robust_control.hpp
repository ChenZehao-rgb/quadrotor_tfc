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
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <cmath>
#include <lib/mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>

#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/robust_control_position_error.h>
#include <uORB/topics/robust_control_velocity_error.h>
#include <uORB/topics/robust_control_attitude_error.h>
#include <uORB/topics/robust_control_rate_error.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/robust_control_data.h>
#include <uORB/topics/robust_control_data_pid.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>

#include <ControlMath.hpp>

using namespace time_literals;

class RobustControl : public ModuleBase<RobustControl>, public ModuleParams
{
public:
    RobustControl() : ModuleParams(nullptr) {}
    ~RobustControl() override {}

    int main();
    
    static px4::AppState appState;

private:

    void	parameters_update();
    matrix::Vector3f Constrain(const matrix::Vector3f &x, const matrix::Vector3f &maxnum);

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::RC_VEL_MAX_X>) _param_rc_vel_x_max,
        (ParamFloat<px4::params::RC_VEL_MAX_Y>) _param_rc_vel_y_max,
        (ParamFloat<px4::params::RC_VEL_MAX_Z>) _param_rc_vel_z_max,
        (ParamFloat<px4::params::RC_ACC_MAX_X>) _param_rc_acc_x_max,
        (ParamFloat<px4::params::RC_ACC_MAX_Y>) _param_rc_acc_y_max,
        (ParamFloat<px4::params::RC_ACC_MAX_Z>) _param_rc_acc_z_max,
        (ParamFloat<px4::params::RC_ATT_MAX_R>) _param_rc_att_r_max,
        (ParamFloat<px4::params::RC_ATT_MAX_P>) _param_rc_att_p_max,
        (ParamFloat<px4::params::RC_ATT_MAX_Y>) _param_rc_att_y_max,
        (ParamFloat<px4::params::RC_RAT_MAX_R>) _param_rc_rat_r_max,
        (ParamFloat<px4::params::RC_RAT_MAX_P>) _param_rc_rat_p_max,
        (ParamFloat<px4::params::RC_RAT_MAX_Y>) _param_rc_rat_y_max,
        (ParamFloat<px4::params::RC_RAC_MAX_R>) _param_rc_rac_r_max,
        (ParamFloat<px4::params::RC_RAC_MAX_P>) _param_rc_rac_p_max,
        (ParamFloat<px4::params::RC_RAC_MAX_Y>) _param_rc_rac_y_max,
        (ParamFloat<px4::params::RC_GRAV>)      _param_rc_grav,
        (ParamFloat<px4::params::RC_MASS>)      _param_rc_mass,
        (ParamFloat<px4::params::RC_KP_P>)      _param_rc_kp_p,
        (ParamFloat<px4::params::RC_KV_P>)      _param_rc_kv_p,
        (ParamFloat<px4::params::RC_KA_P>)      _param_rc_ka_p,
        (ParamFloat<px4::params::RC_KAR_P>)     _param_rc_kar_p,
        (ParamFloat<px4::params::RC_KP_P_X>)    _param_rc_kp_p_x,
        (ParamFloat<px4::params::RC_KP_P_Y>)    _param_rc_kp_p_y,
        (ParamFloat<px4::params::RC_KP_P_Z>)    _param_rc_kp_p_z,
        (ParamFloat<px4::params::RC_KV_P_X>)    _param_rc_kv_p_x,
        (ParamFloat<px4::params::RC_KV_P_Y>)    _param_rc_kv_p_y,
        (ParamFloat<px4::params::RC_KV_P_Z>)    _param_rc_kv_p_z,
        (ParamFloat<px4::params::RC_KA_P_R>)    _param_rc_ka_p_r,
        (ParamFloat<px4::params::RC_KA_P_P>)    _param_rc_ka_p_p,
        (ParamFloat<px4::params::RC_KA_P_Y>)    _param_rc_ka_p_y,
        (ParamFloat<px4::params::RC_KAR_P_R>)   _param_rc_kar_p_r,
        (ParamFloat<px4::params::RC_KAR_P_P>)   _param_rc_kar_p_p,
        (ParamFloat<px4::params::RC_KAR_P_Y>)   _param_rc_kar_p_y,
        (ParamFloat<px4::params::RC_K_FF>)      _param_rc_k_ff,
        (ParamFloat<px4::params::RC_K_FB>)      _param_rc_k_fb,
        (ParamFloat<px4::params::RC_START>)     _param_rc_start
    )

    uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // 订阅位置误差话题
    uORB::Subscription _robust_control_position_error_sub {ORB_ID(robust_control_position_error)};
    robust_control_position_error_s _position_error;
    // 订阅速度误差话题
    uORB::Subscription _robust_control_velocity_error_sub{ORB_ID(robust_control_velocity_error)};
    robust_control_velocity_error_s _velocity_error;
    // 订阅姿态误差话题
    uORB::Subscription _robust_control_attitude_error_sub{ORB_ID(robust_control_attitude_error)};
    robust_control_attitude_error_s _attitude_error;
    // 订阅角速度误差话题
    uORB::Subscription _robust_control_rate_error_sub{ORB_ID(robust_control_rate_error)};
    robust_control_rate_error_s _rate_error;
    // 订阅期望加速度话题
    uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
    vehicle_local_position_setpoint_s _vehicle_local_position_setpoint;
    // 订阅期望姿态话题
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    vehicle_attitude_setpoint_s _vehicle_attitude_setpoint;
    // 订阅当前姿态话题
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    vehicle_attitude_s _vehicle_attitude;
    // 订阅前馈力和力矩话题
    uORB::Subscription _robust_control_data_pid_sub{ORB_ID(robust_control_data_pid)};
    robust_control_data_pid_s _feedforward_force_moment;

    // 发布控制数据话题
    robust_control_data_s _robust_control_data = {};
    uORB::PublicationData<robust_control_data_s> _robust_control_data_pub {ORB_ID(robust_control_data)};

    matrix::Vector<float, 3>        _velocity_max;
    matrix::Vector<float, 3>        _acceleration_max;
    matrix::Vector<float, 3>        _attitude_max;
    matrix::Vector<float, 3>        _rate_max;
    matrix::Vector<float, 3>        _rateacceleration_max;
    matrix::Vector<float, 3>        _pos_error;
    matrix::Vector<float, 3>        _vel_error;
    matrix::Vector<float, 3>        _att_error;
    matrix::Vector<float, 3>        _rat_error;
    matrix::Vector<float, 3>        _acc_sp;
    matrix::Vector<float, 3>        _att_sp;
    matrix::Vector<float, 3>        _rat_sp;
    matrix::Vector<float, 3>        _rat; // 实际角速度
    matrix::Vector<float, 3>        _att;
    matrix::Matrix<float, 6, 12>    _K;      // 大的增益矩阵
    matrix::Matrix<float, 3, 6>     _K_pos;  // 位置控制增益
    matrix::Matrix<float, 3, 6>     _K_att;  // 姿态控制增益
    matrix::Vector<float, 6>        _pos_err; // 存储位置速度误差
    matrix::Vector<float, 6>        _att_err; // 存储姿态角速度误差
    matrix::Vector<float, 3>        v1; // 虚拟控制量v1
    matrix::Vector<float, 3>        v2; // 虚拟控制量v2

    matrix::Matrix3f W, dot_W;

    // 增益定义
    float _KpP;
    float _KvP;
    float _KaP;
    float _KarP;
    float _Kff;
    float _Kfb;
    matrix::Vector<float, 3>        _KpP_xyz;
    matrix::Vector<float, 3>        _KvP_xyz;
    matrix::Vector<float, 3>        _KaP_rpy;
    matrix::Vector<float, 3>        _KarP_rpy;

    matrix::Vector<float, 4>        _feedforward_force_moment_vector;

};
