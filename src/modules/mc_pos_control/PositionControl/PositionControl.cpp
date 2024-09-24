/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix; // 数学库命名空间，提高矩阵和向量运算功能

// 设置速度控制器PID增益
void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

// 设置速度限幅
void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

// 设置升力限幅
void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	// 为防止升力过小，导致无法推算出飞行姿态
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

// 设置水平升力余量
void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

// 更新悬停升力
void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	if (hover_thrust_new > FLT_EPSILON) 
	{
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		setHoverThrust(hover_thrust_new);
	}
}

// 设置无人机的当前状态
void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

// 设置无人机的期望状态
void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint) // setpoint是一个vehicle_local_position_setpoint_s对象
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

// 更新控制器
// update： 控制器的主更新函数，控制器在每个循环周期内调用它来更新状态
// dt 为时间步长
bool PositionControl::update(const float dt)
{
	bool valid = _inputValid(); // 检查输入是否有效，无效则不进行控制器更新

	if (valid) // 如果输入有效，调用 _positionControl() 和 _velocityControl(dt) 来执行位置和速度控制
	{
		_positionControl(); // 位置控制
		_velocityControl(dt); // 速度控制

		// PX4_ISFINITE 检查 _yawspeed_sp 和 _yaw_sp 是否有极值
		// 若 _yawspeed_sp 存在极值，则设为0
		// 若 _yaw_sp 存在极值，则设置为当前值
		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output accleration and thrust setpoint otherwise something went wrong
	// 再次检查输出的加速度设定值 _acc_sp 和 _thr_sp 各个分量是否有极限值，若有则整个控制更新视为无效
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));

	return valid;
}

// 位置控制器
void PositionControl::_positionControl()
{
	// P-position controller
	// _pos_sp - _pos: 计算当前位置 _pos 和目标位置 _pos_sp 之间的误差，即位置偏差。偏差越大，系统需要的修正速度越大。
	// emult(_gain_pos_p): 位置误差与位置控制增益 _gain_pos_p 元素乘法（emult），即比例控制。
	// P 控制器的核心思想是根据误差的大小直接计算输出，控制器增益决定了误差变化引起的速度设定点变化幅度。
	// 这个操作得到的 vel_sp_position 是位置控制器输出的速度设定值，它是通过位置误差乘以增益计算得到的。
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	/* 处理 NaN 值，防止不合法的数值影响控制计算
	addIfNotNanVector3f: 如果 vel_sp_position 中的元素不是 NaN（Not-a-Number，表示无效数值），
	则将 vel_sp_position 添加到当前的速度设定点 _vel_sp 上。 
	作用: 防止 NaN 值影响速度设定点的计算。NaN 通常表示传感器数据错误或计算失败，因此应排除这些无效数据。*/
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	/* setZeroIfNanVector3f: 将 vel_sp_position 中的任何 NaN 值重置为零。
	作用: 确保所有数值都是有效的，防止后续的控制算法使用到无效的 NaN 值。如果有 NaN 值，该位置控制输出会被认为是 0。 */
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	/* 约束水平速度设定点，优先处理目标位置方向的速度分量
	vel_sp_position.xy(): 提取位置控制器输出的水平速度（X 和 Y 方向）。
	_vel_sp - vel_sp_position: 提取速度设定点中的前馈速度分量。
	_lim_vel_horizontal: 设定的水平速度限制。
	constrainXY: 该函数优先保留目标位置设定点（vel_sp_position.xy()）上的速度分量，
	并限制水平速度的大小不超过最大水平速度 _lim_vel_horizontal。当总速度超出限制时，会优先保留位置方向的速度。 */
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	/* 约束垂直速度设定点
	_vel_sp(2): 表示速度设定点中的垂直方向（Z 轴）速度。
	math::constrain: 将垂直速度限制在设定的上下限之间，- _lim_vel_up 表示最大上升速度，_lim_vel_down 表示最大下降速度。 */
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

//  PID 速度控制器：计算速度误差并生成加速度设定点
void PositionControl::_velocityControl(const float dt)
{
	// PID velocity control
	/* _vel_sp - _vel: 计算目标速度 _vel_sp 和当前速度 _vel 之间的误差 vel_error。
	这是 PID 控制器的输入，误差越大，调整的加速度设定点越大。 */
	Vector3f vel_error = _vel_sp - _vel;
	/* vel_error.emult(_gain_vel_p): 速度误差通过 P 控制器的比例增益 _gain_vel_p 进行调节，计算出速度误差对应的加速度。
	_vel_int: 积分控制器部分，积累过去的误差，修正稳态误差。
	_vel_dot.emult(_gain_vel_d): 使用 D 控制器对速度变化率（加速度）进行抑制，避免系统过冲。
	_vel_dot 是速度的导数，表示当前加速度，_gain_vel_d 是微分增益。
	最终结果 acc_sp_velocity 是基于 PID 控制器 计算出来的加速度设定点。 */
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	/* 处理 NaN 值，防止无效数据影响控制计算
	addIfNotNanVector3f: 将计算得到的加速度设定点 acc_sp_velocity 添加到当前加速度设定点 _acc_sp，
	但只有当它不是 NaN 时才执行此操作。这样确保无效的 NaN 数据不会干扰控制器输出。 */
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	// 执行加速度控制
	_accelerationControl();

	// Integrator anti-windup in vertical direction
	/* 垂直方向升力的抗饱和处理
	_thr_sp(2) 是垂直方向的升力。
	vel_error(2) 是垂直方向的速度误差。
	条件中，如果推力在最小值和最大值附近（接近饱和），且垂直速度误差方向与推力方向一致（过大或过小），
	那么将垂直误差 vel_error(2) 设置为 0，防止积分项继续积累。 */
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) 
	{
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	/* thrust_sp_xy: 提取水平升力（X 和 Y 方向的升力分量）。
	thrust_sp_xy_norm: 计算水平推力的模长。
	thrust_max_squared: 推力的最大允许值的平方，用来约束推力的总大小。 */
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	/* 确定在保持水平推力余量的情况下可分配的最大垂直推力
	allocated_horizontal_thrust: 计算在保持水平推力限制（_lim_thr_xy_margin）的情况下分配的水平推力。
	如果当前水平推力大于允许的最大水平推力余量，使用最大余量。
	thrust_z_max_squared: 计算剩余可用于垂直方向的推力值（最大推力减去已分配给水平推力的部分），用于约束垂直推力。 */
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	/* 限制最大垂直推力
	通过 math::max 函数，将当前的垂直推力设定点 _thr_sp(2) 限制在剩余的垂直推力范围内，确保总推力不超过允许的最大推力。 */
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	/* 确定剩余的水平推力
	thrust_max_xy_squared: 计算垂直推力分配后，剩余的可分配给水平推力的总量。 */
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	// thrust_max_xy: 取其平方根得到剩余的最大水平推力，如果剩余的推力值大于 0，则赋值为该值。
	if (thrust_max_xy_squared > 0) 
	{
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	/* 限制水平推力
	如果当前水平推力超过了允许的最大水平推力值 thrust_max_xy，将其按比例缩放到最大允许值，
	确保推力的总大小在合理范围内。 */
	if (thrust_sp_xy_norm > thrust_max_xy) 
	{
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	/* 积分抗饱和控制：在水平方向上追踪饱和情况
	抗饱和控制: 当推力输出达到饱和时，积分项被用来“解除”输出的饱和状态。
	Anti-Windup 技术防止在控制器输出达到极限时，积分项过度积累，导致系统不稳定。
	acc_sp_xy_limited: 通过推力计算出有限的水平加速度设定点。
	arw_gain: 抗饱和增益，用于控制抗饱和修正的幅度。
	这段代码通过减去抗饱和控制的修正项来修正水平误差。 */
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	/* 确保没有 NaN 值
	防止 vel_error 包含 NaN 值，确保后续计算的安全性。任何 NaN 值都会被置为 0。 */
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	/* 更新速度控制的积分项
	将经过修正的速度误差乘以积分增益 _gain_vel_i，并乘以时间步长 dt，更新速度控制器的积分项 _vel_int。 */
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	/* 限制积分项的大小
	防止积分项过大，尤其在垂直方向。
	将垂直方向的积分项限制在 CONSTANTS_ONE_G（重力加速度）范围内，避免积分器失控。 */
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
}

// 加速度控制
void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	/* 假设垂直方向的标准重力加速度用于姿态生成
	创建一个表示机体Z轴方向的向量 body_z，表示期望的机身朝向。
	-_acc_sp(0) 和 -_acc_sp(1) 是加速度设定值在水平轴（X和Y轴）的分量，用来表示期望的加速度方向。
	CONSTANTS_ONE_G 是标准重力加速度（约9.81 m/s²），用于表示Z轴的向上加速度。
	normalized() 将向量归一化，使其为单位向量，表示方向，不考虑大小。 */
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	/* 限制 body_z 向量的倾斜角度
	通过 ControlMath::limitTilt() 函数限制机身的倾斜角度。
	Vector3f(0, 0, 1) 是参考向上的单位向量，表示垂直向上（地面法线）。
	_lim_tilt 是倾斜角度的限制，防止无人机倾斜过大以确保飞行稳定。
	该函数限制 body_z 与参考垂直方向之间的倾斜角度，确保不会超过允许的范围。 */
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	/* 假设悬停推力产生标准重力，缩放推力
	计算垂直方向上的总推力，假设悬停推力能够抵消重力。
	_acc_sp(2) 是加速度设定值的Z轴分量，表示需要的垂直方向加速度。
	(_hover_thrust / CONSTANTS_ONE_G) 相当于无人机质量m。
	减去 _hover_thrust 的目的是从悬停状态进行调整，得到实际所需的推力量。 */
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	/* 将推力投影到预期的机身姿态上
	将计算出的推力投影到机身的实际姿态上。
	Vector3f(0, 0, 1) 是垂直向上的参考向量，和 body_z 向量做点积，得到机体姿态与重力方向之间的关系。
	通过点积运算，计算出推力在实际姿态下需要的调整，以适应当前机身方向。 */
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	/* 确保推力不低于最小推力限制
	将总推力与最小推力限制进行比较，确保推力不会小于设定的最小值。
	-_lim_thr_min 是推力的最小值，确保推力不至于过小，避免无人机失去控制。 */
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	/* 设置推力设定值
	将计算出的推力向量赋值给 _thr_sp，表示推力的方向和大小。
	body_z 是期望的机身朝向向量，collective_thrust 是总推力的大小。
	_thr_sp 代表推力设定值，提供给姿态控制器使用，以控制无人机的飞行姿态。 */
	_thr_sp = body_z * collective_thrust;
}

// 用于检查输入的控制设定点（position, velocity, acceleration）以及飞行器状态（例如位置信息和速度）的有效性
bool PositionControl::_inputValid()
{
	// 初始化一个布尔变量 valid，它用于存储输入检查的结果，初始值设为 true
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	// 使用一个循环检查三个方向轴：x, y, z（索引0, 1, 2）。在每个方向轴上，检查位置、速度和加速度设定点的有效性
	for (int i = 0; i <= 2; i++) 
	{
		/* 在每个方向轴上（x, y, z），检查以下条件之一是否为true：位置设定点 _pos_sp(i)、速度设定点 _vel_sp(i)、
		或加速度设定点 _acc_sp(i) 是否是有限值（不是无穷大或者NaN）。
		PX4_ISFINITE() 用于检查给定值是否为有效的数字
		如果所有设定点（位置、速度、加速度）都无效，则 valid 变为 false。
		换句话说，在每个轴上，至少一个设定点必须是有效的。 */
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	/* 检查x和y轴的期望状态信息是否成对出现。如果x轴的位置信息是有限的，那么y轴的位置信息也应该是有限的，反之亦然。
	这确保了在控制过程中，x和y轴的设定点始终成对出现，防止出现只在一个方向上设置目标位置的情况。 */
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

/* 将当前的局部位置设定点和其他控制信息复制到一个 vehicle_local_position_setpoint_s 结构体中。
这通常是在控制器需要将当前的控制设定点输出给其他模块或子系统时使用的。 */
void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	/* 调用 _acc_sp 的 copyTo() 函数，将加速度设定点 _acc_sp 复制到 local_position_setpoint.acceleration */
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	/* 调用 _thr_sp 的 copyTo() 函数，将推力设定点 _thr_sp 复制到 local_position_setpoint.thrust
	_thr_sp 也是一个三维向量，表示飞行器在 x, y, z 方向上的推力设定值 */
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

/* 定义了 PositionControl::getAttitudeSetpoint() 函数，用于计算并输出飞行器的姿态设定点，
并将其写入传入的 vehicle_attitude_setpoint_s 结构体中 */
/* 这是一个常量成员函数，表示它不会修改类的状态。
该函数接受一个 vehicle_attitude_setpoint_s 类型的引用参数 attitude_setpoint，
用于存储飞行器的姿态设定值。 */
void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	/* 这一行调用了 ControlMath 类中的静态函数 thrustToAttitude()，该函数用于将推力向量和偏航角转换为姿态（即飞行器的角度）。
	thrustToAttitude() 的作用是根据推力向量 _thr_sp 和偏航角 _yaw_sp 计算飞行器需要的姿态（俯仰角、横滚角等），
	并将结果存储在 attitude_setpoint 中。 */
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
