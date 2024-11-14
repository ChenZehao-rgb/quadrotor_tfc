/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "MulticopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

/* 这是 PX4 中 MulticopterPositionControl 类的构造函数，负责初始化与多旋翼无人机位置控制相关的参数和模块。
在这个构造函数中，包含了模块、参数、发布者等的初始化工作。 */
MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	/* SuperBlock 是一种 PX4 中的类，用来处理模块的参数管理。
	构造时会将其名称设置为 "MPC"，这是多旋翼位置控制（Multicopter Position Control）的简称。 */
	SuperBlock(nullptr, "MPC"),
	/* ModuleParams 是一个管理参数的基类，nullptr 表示不需要绑定特定的父模块。 */
	ModuleParams(nullptr),
	/* ScheduledWorkItem 负责在工作队列中调度任务。
	这里指定了模块的名称 (MODULE_NAME) 和
	一个与导航和控制器相关的工作队列配置 (px4::wq_configurations::nav_and_controllers)，
	保证这个模块在适当的工作队列中执行。 */
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	/* 初始化姿态设定点的发布者 ORB_ID(vehicle_attitude_setpoint)。
	如果 vtol 为 true，表示这是垂直起降固定翼 (VTOL)，使用虚拟的姿态设定点 mc_virtual_attitude_setpoint，
	否则使用标准的 vehicle_attitude_setpoint。 */
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	/* 初始化 _vel_x_deriv，这是用于计算 X 轴速度导数的模块，参数名称是 "VELD"。它在位置控制过程中会使用到。 */
	_vel_x_deriv(this, "VELD"),
	/* 初始化 Y 轴的速度导数 */
	_vel_y_deriv(this, "VELD"),
	/* 初始化 Z 轴的速度导数 */
	_vel_z_deriv(this, "VELD")
{
	/* 更新所有的参数，true 表示强制进行参数更新。在这个模块中，位置控制所需的参数会被加载或更新。 */
	parameters_update(true);
	/* 设置自动降落时的时间滞后 LOITER_TIME_BEFORE_DESCEND，确保在紧急情况下不会立即触发降落，
	而是经过一段时间延迟（即滞后时间）。 */
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
	/* 设置倾斜角度限制的变化率，这样可以防止倾斜角度变化过快，提供更平滑的控制。 */
	_tilt_limit_slew_rate.setSlewRate(.2f);
	/* 将设定点 _setpoint 重置为 NaN（非数字），表示当前没有有效的目标设定点。 */
	reset_setpoint_to_nan(_setpoint);
	/* 广播（发布）起飞状态，确保这个模块能够向其他模块（如地面站）发布起飞相关的信息。 */
	_takeoff_status_pub.advertise();
}

// 这是 MulticopterPositionControl 类的析构函数，用于在对象被销毁时释放资源。
/* 析构函数是当对象生命周期结束时被调用的函数，用于清理和释放资源。在 C++ 中，析构函数名是类名之前加一个波浪符号 ~。 */
MulticopterPositionControl::~MulticopterPositionControl()
{
	/* perf_free() 是一个性能监控工具函数，用来释放通过 perf_alloc_* 函数分配的性能计数器。
	_cycle_perf 是一个性能计数器，用于记录控制回路的执行时间或频率。
	通过 perf_free() 释放它时，可以避免内存泄漏，并结束性能统计记录。 */
	perf_free(_cycle_perf);
}

//  MulticopterPositionControl 类的初始化函数，用来初始化对象和启动调度机制
bool MulticopterPositionControl::init()
{
	/* _local_pos_sub: 这是一个订阅本地位置的对象（通常订阅的是 vehicle_local_position 主题），
	负责接收飞行器的位置信息。
	registerCallback(): 该函数注册了一个回调函数，当新的位置信息发布时会自动调用回调函数处理数据。
	它通常与 PX4 的消息传递系统（ORB）配合使用。 */
	if (!_local_pos_sub.registerCallback()) 
	{
		/* 失败处理: 如果回调注册失败，代码会打印错误信息 "callback registration failed"，
		并返回 false，表示初始化失败。 */
		PX4_ERR("callback registration failed");
		return false;
	}

	/* hrt_absolute_time(): 获取当前的高分辨率时间戳，单位通常为微秒。这个函数会记录当前的系统时间，
	用于控制循环的时间管理（比如计算每个控制周期的时间差）。
	_time_stamp_last_loop: 保存上一次控制循环执行的时间戳，用于在后续的控制逻辑中计算时间差，
	以确保控制器按预定的时间间隔执行。 */
	_time_stamp_last_loop = hrt_absolute_time();
	/* 这个函数会立即调度该模块的任务，使得 PX4 的任务调度系统可以安排该模块的运行。
	它通过 PX4 的调度机制将控制器的执行时间加入到任务队列中，从而确保 MulticopterPositionControl 在合适的时刻运行。 */
	ScheduleNow();

	return true;
}

/* 这段代码是 MulticopterPositionControl 类中的 parameters_update 函数，用于更新飞控的参数设置。
它会检查参数是否有更新，然后根据这些更新对飞行器的控制参数进行调整。
函数声明，负责更新控制器参数。force 参数决定是否强制更新。 */
void MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	// 检查参数更新主题是否有新的数据，或是否强制更新（force 参数为 true）
	// 如果有新参数或者强制更新标志为真，进入更新逻辑。
	if (_parameter_update_sub.updated() || force) 
	{
		// clear update
		/* 从参数更新订阅中复制最新的数据到 pupdate。
		通过 _parameter_update_sub.copy() 获取最新的参数更新信息。 */
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		/* 更新模块和父类中的参数（分别为 ModuleParams 和 SuperBlock），即从参数存储中加载最新参数。 */
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		// 定义并初始化一个变量 num_changed，用于记录发生更改的参数数量。
		int num_changed = 0;
		/* 检查 MPC_SYS_VEHICLE_RESP 参数值是否有效（大于等于 0），表示控制响应度。
		如果响应度参数有效，则继续更新与响应度相关的参数。 */
		if (_param_sys_vehicle_resp.get() >= 0.f) 
		{
			// make it less sensitive at the lower end
			/* 将 MPC_SYS_VEHICLE_RESP 参数平方，生成响应度的一个非线性值。
			增加低响应度时的敏感度，使得响应变化更细腻。 */
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			/* 更新横向加速度、最大横向加速度、最大偏航速率等一系列参数，
			使用 commit_no_notification 来执行参数的静默提交，不发送通知。 */
			/* 根据响应度线性插值（math::lerp()），更新不同控制参数，如加速度、最大速率等。 */
			// 更新横向加速度参数，基于响应性从 1 到 15 之间进行线性插值。math::lerp 用于线性插值。
			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			// 更新最大横向加速度，基于响应性从 2 到 15 之间线性插值。
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			// 更新最大手动偏航速度，基于响应性从 80 到 450 之间线性插值。
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			// 如果响应性大于 0.6，执行下述更新。
			if (responsiveness > 0.6f) 
			{
				// 将手动偏航响应时间常数设置为 0，意味着瞬时响应。
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);
			} 
			else 
			{
				// 如果响应性小于或等于 0.6，基于响应性线性插值，将偏航响应时间从 0.5 变为 0。
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			// 如果响应性小于 0.5，将最大飞行倾角设置为 45 度。
			if (responsiveness < 0.5f) 
			{
				// 约束最大飞行倾角为 45 度，保证安全飞行。
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);
			} 
			// 如果响应性大于 0.5，则基于响应性在 45 到 70 度之间插值，并确保最大值不会超过安全角度 MAX_SAFE_TILT_DEG。
			else 
			{
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			// 基于响应性线性插值，更新下降时的最大加速度。
			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			// 基于响应性线性插值，更新上升时的最大加速度。
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			// 基于响应性线性插值，更新最大加加速度（jerk），即加速度变化率。
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			// 更新自动模式下的最大加加速度。
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		// 检查是否有全局的XY轴速度参数 _param_mpc_xy_vel_all 被设置为非负值。
		if (_param_mpc_xy_vel_all.get() >= 0.f) 
		{
			// 获取全局XY速度参数的值并存储在 xy_vel 变量中。
			float xy_vel = _param_mpc_xy_vel_all.get();
			// 将全局的XY速度值赋值给手动速度参数 _param_mpc_vel_manual。
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			// 将全局的XY速度值赋值给巡航速度参数 _param_mpc_xy_cruise。
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			// 将全局的XY速度值赋值给最大XY速度参数 _param_mpc_xy_vel_max。
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		// 检查全局Z轴速度参数是否设置为非负值。
		if (_param_mpc_z_vel_all.get() >= 0.f) 
		{
			// 获取全局的Z轴速度参数值并存储在 z_vel 变量中。
			float z_vel = _param_mpc_z_vel_all.get();
			// 更新自动上升速度参数 _param_mpc_z_v_auto_up，赋值为全局Z轴速度
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			// 更新最大上升速度参数 _param_mpc_z_vel_max_up，赋值为全局Z轴速度。
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			// 更新自动下降速度参数 _param_mpc_z_v_auto_dn，其值为全局Z轴速度的75%。
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			// 更新最大下降速度参数 _param_mpc_z_vel_max_dn，其值为全局Z轴速度的75%。
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			// 更新起飞速度参数 _param_mpc_tko_speed，其值为全局Z轴速度的60%。
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			// 更新降落速度参数 _param_mpc_land_speed，其值为全局Z轴速度的50%。
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		// 如果有参数发生了更改 (num_changed > 0)，则执行通知。
		if (num_changed > 0) 
		{
			// 调用函数 param_notify_changes() 通知系统参数已更改。
			param_notify_changes();
		}

		// 检查飞行时的最大倾角 _param_mpc_tiltmax_air 是否超过了安全倾角值 MAX_SAFE_TILT_DEG。
		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) 
		{
			// 如果超过安全值，将最大倾角设置为安全倾角 MAX_SAFE_TILT_DEG。
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			// 将新的倾角限制值应用到参数系统中。
			_param_mpc_tiltmax_air.commit();
			// 使用 MAVLink 记录一条关键日志，通知用户倾角已经被限制到安全值。
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			// 使用 events 模块发送一个事件，通知系统用户最大倾角被限制在安全值。
			events::send<float>(events::ID("mc_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		// 检查着陆时的最大倾角是否超过飞行时的最大倾角。
		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) 
		{
			// 如果着陆倾角超出飞行倾角限制，将其设置为飞行时的最大倾角。
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			// 应用新的着陆倾角限制。
			_param_mpc_tiltmax_lnd.commit();
			// 记录一条关键日志，通知用户着陆倾角已被飞行倾角限制。
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			// 使用 events 模块发送通知，提醒用户着陆倾角已被飞行倾角限制。
			events::send<float>(events::ID("mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		/* 设置位置控制器的增益参数。通过将XY轴的位置增益 _param_mpc_xy_p 和Z轴的位置增益 _param_mpc_z_p 作为参数，
		设置飞行器在不同轴向上的位置控制响应强度。使用 Vector3f 表示三个方向（X, Y, Z）的增益。 */
		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		// 设置速度控制器的增益参数。
		_control.setVelocityGains(
			/* 为XY轴和Z轴的速度P增益，即速度误差对控制信号的直接比例调节，
			取自 _param_mpc_xy_vel_p_acc 和 _param_mpc_z_vel_p_acc。 */
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			/* 为XY轴和Z轴的速度I增益，即速度误差的积分调节，取自 _param_mpc_xy_vel_i_acc 和 _param_mpc_z_vel_i_acc。 */
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			/* 为XY轴和Z轴的速度D增益，即速度误差的导数调节，取自 _param_mpc_xy_vel_d_acc 和 _param_mpc_z_vel_d_acc */
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));
		// 设置水平推力的安全余量，即 _param_mpc_thr_xy_marg 参数，确保飞行器在XY方向的推力不超过一定限度。
		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());

		// Check that the design parameters are inside the absolute maximum constraints
		// 检查巡航速度是否超出最大速度限制
		// 检查巡航速度 _param_mpc_xy_cruise 是否超过最大允许的XY速度 _param_mpc_xy_vel_max
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) 
		{
			// 如果超出限制，将巡航速度设置为最大允许的XY速度。
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			// 应用更新后的巡航速度参数。
			_param_mpc_xy_cruise.commit();
			// 使用MAVLink发送关键日志，通知用户巡航速度已被限制在安全值。
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			// 通过 events 系统发送事件通知，提示巡航速度已被约束。
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		// 检查手动速度是否超出最大速度限制
		// 检查手动速度 _param_mpc_vel_manual 是否超过最大允许的XY速度 _param_mpc_xy_vel_max。
		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) 
		{
			// 如果超出限制，将手动速度设置为最大允许的XY速度。
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			// 应用更新后的手动速度参数。
			_param_mpc_vel_manual.commit();
			// 使用MAVLink发送关键日志，通知用户手动速度已被限制在安全值。
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			// 通过 events 系统发送事件通知，提示手动速度已被约束。
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		// 检查自动上升速度是否超出最大上升速度限制
		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) 
		{
			// 如果超出限制，将自动上升速度设置为最大允许的上升速度。
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			// 应用更新后的悬停推力参数。
			_param_mpc_z_v_auto_up.commit();
			// 记录MAVLink日志，通知用户悬停推力已被限制在允许范围内。
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			// 通过 events 系统发送事件通知，告知悬停推力已被约束。
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		// 检查自动下降速度是否超出最大下降速度限制
		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) 
		{
			// 如果超出限制，将自动下降速度设置为最大允许的下降速度。
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			// 应用更新后的自动下降速度参数。
			_param_mpc_z_v_auto_dn.commit();
			// 记录一条MAVLink关键日志，通知用户下降速度已被限制。
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			// 发送事件通知，告知用户下降速度已被约束。
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		// 检查悬停推力是否超出最小/最大推力限制
		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) 
		{
			// 如果超出限制，通过 math::constrain 函数将悬停推力约束在最小推力和最大推力之间。
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			// 应用更新后的悬停推力参数。
			_param_mpc_thr_hover.commit();
			// 记录MAVLink日志，通知用户悬停推力已被限制在允许范围内。
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			// 通过 events 系统发送事件通知，告知悬停推力已被约束。
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		// 检查是否需要设置悬停推力
		/* _param_mpc_use_hte.get()：是否启用基于悬停推力估计的控制，如果未启用，则进入代码块。
		_hover_thrust_initialized：是否已经初始化了悬停推力，如果尚未初始化，则进入代码块。 */
		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) 
		{
			// 将控制器的悬停推力设置为参数中存储的悬停推力值（_param_mpc_thr_hover）。
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			// 标记悬停推力已经初始化。
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		// 初始化起飞和降落速度参数并应用约束
		/* 将起飞速度参数 _param_mpc_tko_speed 限制在最大上升速度 _param_mpc_z_vel_max_up 之内。
		即起飞速度不能超过上升速度的上限。 */
		/*  将降落速度参数 _param_mpc_land_speed 限制在最大下降速度 _param_mpc_z_vel_max_dn 之内。
		即降落速度不能超过下降速度的上限。 */
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		/* 设置电机启动时间
		将起飞过程中电机启动的时间设为参数 _param_mpc_spoolup_time 中定义的值。 */ 
		_takeoff.setSpoolupTime(_param_mpc_spoolup_time.get());
		/* 设置起飞斜坡时间
		将起飞推力斜坡（逐步增大推力的过程）的持续时间设置为参数 _param_mpc_tko_ramp_t 中定义的值。 */
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		/* 生成起飞初始斜坡值
		使用垂直速度的比例增益（_param_mpc_z_vel_p_acc）来生成初始的起飞斜坡值。这会影响起飞时推力的变化速率。 */
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}
}

/* 该函数 set_vehicle_states 负责将来自 vehicle_local_position_s 结构中的位置和速度数据转换为 PositionControlStates 结构，
并在数据无效或缺失时，将相关状态设置为 NAN（Not A Number）。 */
PositionControlStates MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s &local_pos)
{
	PositionControlStates states;

	// only set position states if valid and finite
	if (PX4_ISFINITE(local_pos.x) && PX4_ISFINITE(local_pos.y) && local_pos.xy_valid) {
		states.position(0) = local_pos.x;
		states.position(1) = local_pos.y;

	} else {
		states.position(0) = NAN;
		states.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		states.position(2) = local_pos.z;

	} else {
		states.position(2) = NAN;
	}

	if (PX4_ISFINITE(local_pos.vx) && PX4_ISFINITE(local_pos.vy) && local_pos.v_xy_valid) {
		states.velocity(0) = local_pos.vx;
		states.velocity(1) = local_pos.vy;
		states.acceleration(0) = _vel_x_deriv.update(local_pos.vx);
		states.acceleration(1) = _vel_y_deriv.update(local_pos.vy);

	} else {
		states.velocity(0) = NAN;
		states.velocity(1) = NAN;
		states.acceleration(0) = NAN;
		states.acceleration(1) = NAN;

		// reset derivatives to prevent acceleration spikes when regaining velocity
		_vel_x_deriv.reset();
		_vel_y_deriv.reset();
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		states.velocity(2) = local_pos.vz;
		states.acceleration(2) = _vel_z_deriv.update(states.velocity(2));

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset derivative to prevent acceleration spikes when regaining velocity
		_vel_z_deriv.reset();
	}

	states.yaw = local_pos.heading;

	return states;
}

// 控制多旋翼飞行器的飞行状态
// 定义 Run 方法，这是控制器的主循环，用于处理控制逻辑和更新状态。
// 每个周期都会执行该方法来更新飞行控制状态。
void MulticopterPositionControl::Run()
{
	// 检查控制器是否应退出。如果是，则注销本地位置订阅者的回调，清理资源，并返回
	// 如果系统收到退出请求，注销本地位置的回调函数，并调用 exit_and_cleanup() 方法清理资源，随后退出当前函数。
	if (should_exit()) 
	{
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	// 安排下一次调用此方法的时间，延迟 100 毫秒后再调用 Run() 方法。
	ScheduleDelayed(100_ms);

	// 更新控制器的参数。false 表示仅更新有变动的参数，而不是强制刷新所有参数。
	parameters_update(false);

	// 开始性能监控计时，用于记录这个周期的执行时间，性能数据将用于调试和优化。
	perf_begin(_cycle_perf);
	// 定义 local_pos 变量，用于存储飞行器的本地位置数据。
	vehicle_local_position_s local_pos;

	// 检查是否有新的本地位置数据可用，如果有，更新 local_pos 数据结构
	if (_local_pos_sub.update(&local_pos)) 
	{
		// 获取当前更新的时间戳，并将其存储在 time_stamp_now 变量中
		const hrt_abstime time_stamp_now = local_pos.timestamp_sample;
		// 计算当前周期与上一个周期的时间差（以秒为单位）。
		// math::constrain 函数限制该时间差在 0.002 到 0.04 秒之间，防止时间差过大或过小。
		const float dt = math::constrain(((time_stamp_now - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		// 更新最后一次循环的时间戳，以便在下一个周期计算新的时间差。
		_time_stamp_last_loop = time_stamp_now;

		// set _dt in controllib Block for BlockDerivative
		// 将 dt 设置为控制库的时间增量，用于计算系统导数（例如速度的导数）。
		setDt(dt);
		// 将故障安全模式标志 _in_failsafe 设为 false，表示系统目前不在故障安全模式下。
		_in_failsafe = false;

		// 更新车辆控制模式和着陆检测数据，从订阅的数据流中获取最新信息。
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		// 如果启用了悬停推力估计（由参数 _param_mpc_use_hte 决定），那么检查并更新悬停推力的估计值。
		// 如果估计值有效，则更新控制器的悬停推力。
		if (_param_mpc_use_hte.get()) 
		{
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		// 调用 set_vehicle_states() 方法，根据当前本地位置数据来设置飞行器的状态，并将其存储在 states 变量中。
		PositionControlStates states{set_vehicle_states(local_pos)};

		// 如果当前启用了多旋翼飞行器的位置控制模式，则进入这个代码块以执行位置控制。
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) 
		{

			// 检查并更新轨迹设定点（目标位置、速度等），并记录是否有新的轨迹设定点可用。
			const bool is_trajectory_setpoint_updated = _trajectory_setpoint_sub.update(&_setpoint);

			// adjust existing (or older) setpoint with any EKF reset deltas
			// 如果设定点的时间戳早于当前位置的时间戳，进入此代码块进行调整。这种情况通常发生在惯性导航系统（EKF）重置之后。
			if (_setpoint.timestamp < local_pos.timestamp) 
			{
				// 如果 X 和 Y 轴速度重置计数器发生了变化，调整设定点的 X 和 Y 轴速度，使得设定点与位置估计保持一致。
				if (local_pos.vxy_reset_counter != _vxy_reset_counter) 
				{
					_setpoint.vx += local_pos.delta_vxy[0];
					_setpoint.vy += local_pos.delta_vxy[1];
				}

				// 如果 Z 轴速度重置计数器发生了变化，调整设定点的 Z 轴速度。
				if (local_pos.vz_reset_counter != _vz_reset_counter) 
				{
					_setpoint.vz += local_pos.delta_vz;
				}

				// 如果 X 和 Y 轴位置重置计数器发生了变化，调整设定点的 X 和 Y 轴位置。
				if (local_pos.xy_reset_counter != _xy_reset_counter) 
				{
					_setpoint.x += local_pos.delta_xy[0];
					_setpoint.y += local_pos.delta_xy[1];
				}

				// 如果 Z 轴位置重置计数器发生了变化，调整设定点的 Z 轴位置。
				if (local_pos.z_reset_counter != _z_reset_counter) 
				{
					_setpoint.z += local_pos.delta_z;
				}

				// 如果航向重置计数器发生了变化，调整设定点的航向角（偏航角）。
				if (local_pos.heading_reset_counter != _heading_reset_counter) 
				{
					_setpoint.yaw += local_pos.delta_heading;
				}
			}

			// update vehicle constraints and handle smooth takeoff
			// 更新飞行器的约束条件（例如最大速度、加速度等），并处理平稳起飞的逻辑。
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			// 确保起飞速度的上升斜坡值合理，避免因 NaN 值或过高的值导致起飞过程卡住。这里限制了速度的最大值。
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) 
			{
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			// 如果飞行器处于离线控制模式（例如通过外部计算机控制），进入此代码块。
			if (_vehicle_control_mode.flag_control_offboard_enabled) 
			{
				
				// 检查是否需要起飞的条件：飞行器已解锁且当前着陆状态，并且设定点的时间戳在 1 秒以内。
				bool want_takeoff = _vehicle_control_mode.flag_armed && _vehicle_land_detected.landed
						    && hrt_elapsed_time(&_setpoint.timestamp) < 1_s;

				// 根据设定点的高度、速度和加速度来判断是否希望起飞。如果符合条件，将 want_takeoff 设为 true，否则设为 false。
				if (want_takeoff && PX4_ISFINITE(_setpoint.z)
				    && (_setpoint.z < states.position(2))) 
				{

					_vehicle_constraints.want_takeoff = true;

				} 
				else if (want_takeoff && PX4_ISFINITE(_setpoint.vz)
					   && (_setpoint.vz < 0.f)) 
				{

					_vehicle_constraints.want_takeoff = true;

				} 
				else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f))    
				{

					_vehicle_constraints.want_takeoff = true;

				} 
				else 
				{
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				// 使用默认参数覆盖当前的上下速度限制。
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			// handle smooth takeoff
			// 更新起飞状态，处理平稳起飞的逻辑，包括飞行器的解锁状态和着陆检测信息。
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, false, time_stamp_now);

			// 检查飞行器是否已进入飞行状态。
			const bool flying = (_takeoff.getTakeoffState() >= TakeoffState::flight);

			// 如果轨迹设定点更新，并且飞行器尚未起飞，确保加速度前馈项不会影响起飞，并重置悬停推力。
			if (is_trajectory_setpoint_updated) 
			{
				// make sure takeoff ramp is not amended by acceleration feed-forward
				if (!flying) 
				{
					_setpoint.acceleration[2] = NAN;
					// hover_thrust maybe reset on takeoff
					_control.setHoverThrust(_param_mpc_thr_hover.get());
				}

				const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
				const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);

				if (not_taken_off || flying_but_ground_contact) 
				{
					// we are not flying yet and need to avoid any corrections
					reset_setpoint_to_nan(_setpoint);
					Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

					// prevent any integrator windup
					_control.resetIntegral();
				}
			}

			// limit tilt during takeoff ramupup
			/* 根据飞行器的起飞状态设置最大倾斜角度限制。
			如果飞行器尚未起飞，则限制倾斜角度为地面最大倾斜角度 (_param_mpc_tiltmax_lnd)，
			如果已在飞行，则使用空中最大倾斜角度 (_param_mpc_tiltmax_air)。
			同时，倾斜角度被逐步平滑地调整，调用 _tilt_limit_slew_rate.update 函数更新当前允许的倾斜角度。 */
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			/* 调用 _takeoff.updateRamp() 更新飞行器起飞过程的速度斜坡，返回上升速度。
			若系统提供的上升速度是有效数值 (PX4_ISFINITE)，则使用该速度，
			否则使用参数中的最大上升速度 _param_mpc_z_vel_max_up。 */
			const float speed_up = _takeoff.updateRamp(dt,
					       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			// 类似于上升速度的处理逻辑，检查下降速度是否有效，若无效则使用参数中的最大下降速度 _param_mpc_z_vel_max_dn。
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			// 如果飞行器正在飞行，最小推力值设置为参数中定义的最小推力值 _param_mpc_thr_min；否则，起飞时最小推力为 0。
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;

			// 调用控制器的 setThrustLimits() 方法，设置推力的上下限。
			// 最小推力根据飞行状态决定，最大推力为参数中指定的 _param_mpc_thr_max。
			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			/* 设置飞行器的速度限制，包括水平方向 (_param_mpc_xy_vel_max)、垂直方向的上升速度
			(speed_up 与最大上升速度的较小值) 和下降速度 (speed_down 和 0 的较大值)。
			确保垂直方向的速度不会超过最大值，并防止负速度导致错误。 */
			_control.setVelocityLimits(
				_param_mpc_xy_vel_max.get(),
				math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
				math::max(speed_down, 0.f));

			// 将更新后的设定点 _setpoint 传递给控制器，用作当前控制周期的输入。
			_control.setInputSetpoint(_setpoint);

			// update states
			/* 如果当前设定点中高度值无效（_setpoint.z 非有限值），但是速度设定点（_setpoint.vz）是有效的并且非零，则进入此逻辑块。
			同时需要保证当前 Z 轴速度导数（local_pos.z_deriv）、位置和速度估计值是有效的。 */
			if (!PX4_ISFINITE(_setpoint.z)
			    && PX4_ISFINITE(_setpoint.vz) && (fabsf(_setpoint.vz) > FLT_EPSILON)
			    && PX4_ISFINITE(local_pos.z_deriv) && local_pos.z_valid && local_pos.v_z_valid) 
			{
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				/* 为了平滑地过渡速度设定点，将高度导数 local_pos.z_deriv 和位置估计的速度 local_pos.vz 混合计算，以避免突然变化。
				混合比例由当前设定的速度与着陆速度的比值确定，着陆速度参数为 _param_mpc_land_speed。 */
				float weighting = fminf(fabsf(_setpoint.vz) / _param_mpc_land_speed.get(), 1.f);
				states.velocity(2) = local_pos.z_deriv * weighting + local_pos.vz * (1.f - weighting);
			}

			// 将飞行器的状态（位置、速度等）更新给控制器。
			_control.setState(states);

			// Run position control
			/* 执行位置控制更新，如果更新成功，说明设定点是有效的，系统正常工作，
			则取消故障着陆模式（_failsafe_land_hysteresis 设为 false）。
			如果更新失败，进入 else 代码块处理故障模式。 */
			// updata(dt)函数中包含了位置控制器和速度控制器
			if (_control.update(dt)) 
			{
				_failsafe_land_hysteresis.set_state_and_update(false, time_stamp_now);

			} 
			else 
			{
				// Failsafe
				//  do not warn while we are disarmed, as we might not have valid setpoints yet
				/* 进入故障模式。如果当前已经解锁 (_vehicle_control_mode.flag_armed) 
				并且距离上次警告的时间超过了 2 秒，则触发一个新的故障警告。 */
				const bool warn_failsafe = ((time_stamp_now - _last_warn) > 2_s) && _vehicle_control_mode.flag_armed;

				// 如果触发了故障警告，记录警告信息 "invalid setpoints" 并更新最后一次警告的时间戳。
				if (warn_failsafe) 
				{
					PX4_WARN("invalid setpoints");
					_last_warn = time_stamp_now;
				}

				// 创建一个默认的 failsafe_setpoint（安全模式的本地位置设定点），
				// 并调用 failsafe() 函数更新这个设定点，用于故障保护。
				vehicle_local_position_setpoint_s failsafe_setpoint{};
				failsafe(time_stamp_now, failsafe_setpoint, states, warn_failsafe);

				// reset constraints
				// 重置飞行器的约束条件，将所有限制恢复到默认状态。
				_vehicle_constraints = {0, NAN, NAN, false, {}};

				// 在故障模式下，使用故障保护设定点和默认的速度限制执行控制更新。
				_control.setInputSetpoint(failsafe_setpoint);
				_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());
				_control.update(dt);
			}

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			// 获取并发布控制器生成的内部位置控制设定点（包含 PID 校正后的设定点）。
			// 这些数据会被其他模块（例如着陆检测器）用来判断飞行器的飞行意图。
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// Publish attitude setpoint output
			// 获取并发布控制器生成的姿态设定点（包含偏航角和倾斜角等），用于控制飞行器的姿态。
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} 
		// 如果位置控制模式没有启用，则更新起飞状态，这样即使在非高度控制模式下也能正确跳过起飞状态。
		else 
		{
			// an update is necessary here because otherwise the takeoff state doesn't get skiped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    time_stamp_now);
		}

		// Publish takeoff status
		// 发布起飞状态信息。如果起飞状态或倾斜角度限制发生了变化，则更新并发布这些信息，供其他模块使用。
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());
		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) 
		{
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}

		// save latest reset counters
		// 保存本地位置的重置计数器，这些计数器用于检测位置和速度的重置事件。
		_vxy_reset_counter = local_pos.vxy_reset_counter;
		_vz_reset_counter = local_pos.vz_reset_counter;
		_xy_reset_counter = local_pos.xy_reset_counter;
		_z_reset_counter = local_pos.z_reset_counter;
		_heading_reset_counter = local_pos.heading_reset_counter;
	}

	// 结束当前周期的性能监控计时，记录本次控制循环的执行时间。
	perf_end(_cycle_perf);
}

/* 这段代码是 MulticopterPositionControl 类中处理无人机失效保护（Failsafe）的逻辑。
当无人机遇到系统故障时，系统会进入保护模式，执行安全措施来确保无人机不会失控并安全着陆。 */
/* 该函数的主要功能是在无人机进入失效保护时，控制无人机的行为，确保其能够安全着陆。
具体策略包括停止水平方向运动、根据情况执行降落或紧急下降，并根据传感器的可用性动态调整设定点。
如果有效的传感器数据不可用，系统会以低推力进行盲降，以避免意外失控。 */
void MulticopterPositionControl::failsafe(const hrt_abstime &now, vehicle_local_position_setpoint_s &setpoint,
		const PositionControlStates &states, bool warn)
{
	// Only react after a short delay
	_failsafe_land_hysteresis.set_state_and_update(true, now);

	if (_failsafe_land_hysteresis.get_state()) {
		reset_setpoint_to_nan(setpoint);

		if (PX4_ISFINITE(states.velocity(0)) && PX4_ISFINITE(states.velocity(1))) {
			// don't move along xy
			setpoint.vx = setpoint.vy = 0.f;

			if (warn) {
				PX4_WARN("Failsafe: stop and wait");
			}

		} else {
			// descend with land speed since we can't stop
			setpoint.acceleration[0] = setpoint.acceleration[1] = 0.f;
			setpoint.vz = _param_mpc_land_speed.get();

			if (warn) {
				PX4_WARN("Failsafe: blind land");
			}
		}

		if (PX4_ISFINITE(states.velocity(2))) {
			// don't move along z if we can stop in all dimensions
			if (!PX4_ISFINITE(setpoint.vz)) {
				setpoint.vz = 0.f;
			}

		} else {
			// emergency descend with a bit below hover thrust
			setpoint.vz = NAN;
			setpoint.acceleration[2] = .3f;

			if (warn) {
				PX4_WARN("Failsafe: blind descend");
			}
		}

		_in_failsafe = true;
	}
}

/* 这段代码定义了一个函数 reset_setpoint_to_nan，
用于将无人机的所有位置、速度、加速度、推力等设定点重置为 NAN（Not A Number），
表明这些数据在当前状态下无效。这通常用于清空设定点，尤其是在进入紧急状态（如失效保护）时。
这个操作通常用于进入紧急模式时，让无人机的控制器意识到当前设定点不可用，避免依据无效数据进行不安全的操作。 */
void MulticopterPositionControl::reset_setpoint_to_nan(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = setpoint.y = setpoint.z = NAN;
	setpoint.vx = setpoint.vy = setpoint.vz = NAN;
	setpoint.yaw = setpoint.yawspeed = NAN;
	setpoint.acceleration[0] = setpoint.acceleration[1] = setpoint.acceleration[2] = NAN;
	setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
}

/* 这段代码是 MulticopterPositionControl 类中的 task_spawn 函数，
用于创建并初始化一个新的 MulticopterPositionControl 实例，并将其添加到工作队列中运行。 */
/* task_spawn 函数用于根据输入参数（是否为 VTOL 飞行器）创建并启动一个多旋翼或 VTOL 控制器实例。
它先创建对象并进行初始化，如果成功则将其加入工作队列，并返回成功标志；如果失败则清理资源并返回错误。 */
int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

/* 这段代码是 MulticopterPositionControl 类中的 custom_command 函数，它处理自定义命令输入，
并在无法识别命令时返回帮助信息。 */
int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

/* 这个函数通过打印模块的描述、名称、启动命令、可选参数等信息，向用户提供使用该模块的帮助信息。
如果传入了一个错误原因，则会在描述之前显示警告信息。 */
int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/* mc_pos_control_main 是 MulticopterPositionControl 模块的入口函数，
它被 PX4 系统调用以执行模块的各种命令（例如启动和停止）。
extern "C" 和 __EXPORT 使得这个函数可以被 PX4 的 C 风格代码正确识别和调用。 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
