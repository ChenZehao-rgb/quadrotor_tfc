/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Functions.hpp>

using namespace matrix;

// 该函数用于设置姿态控制器的比例增益 _proportional_gain 和偏航权重 _yaw_w
void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain, const float yaw_weight)
{
	// 将传入的 proportional_gain 值赋给姿态控制器内部的比例增益 _proportional_gain
	_proportional_gain = proportional_gain;
	// 将传入的 yaw_weight 值限制在 [0, 1] 之间，并赋值给 _yaw_w。这确保了偏航控制的权重在合理范围内。
	_yaw_w = math::constrain(yaw_weight, 0.f, 1.f);

	// compensate for the effect of the yaw weight rescaling the output
	// 如果偏航权重 _yaw_w 大于一个非常小的数值，则进行补偿，防止偏航控制受到不必要的放大或缩小影响。
	if (_yaw_w > 1e-4f) 
	{
		// 通过将 _proportional_gain(2)（即 Z 轴方向的增益）除以 _yaw_w，补偿由于偏航权重引起的输出缩放。
		_proportional_gain(2) /= _yaw_w;
	}
}

/* 该函数是姿态控制器的主要更新函数，用于计算无人机的角速度设定值。输入为当前姿态的四元数 q，
输出为角速度设定值 rate_setpoint。 */
matrix::Vector3f AttitudeControl::update(const Quatf &q) const
{
	/* 提取姿态控制器内部保存的期望姿态四元数 _attitude_setpoint_q，并存储在变量 qd 中，作为完整的期望姿态。 */
	Quatf qd = _attitude_setpoint_q;

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	// 从当前姿态四元数 q 中提取旋转矩阵的第三列，即当前姿态在机体坐标系中的 Z 轴向量 e_z。这代表无人机当前朝向的 Z 轴方向。
	const Vector3f e_z = q.dcm_z();
	// 从期望姿态四元数 qd 中提取旋转矩阵的第三列，即期望姿态的 Z 轴向量 e_z_d，代表无人机期望的 Z 轴方向。
	const Vector3f e_z_d = qd.dcm_z();
	// 生成一个只考虑当前 Z 轴与期望 Z 轴之间差异的简化四元数 qd_red，忽略偏航的影响，仅考虑无人机在滚转和俯仰方向上的姿态。
	Quatf qd_red(e_z, e_z_d);

	/* 检查简化的四元数 qd_red 是否接近反向的极限情况（即无人机与期望姿态完全相反）。
	fabsf(qd_red(1)) > (1.f - 1e-5f) 和 fabsf(qd_red(2)) > (1.f - 1e-5f) 是在检查四元数中的 Y 和 Z 分量，
	确保它们没有无限接近 ±1 的情况。 */
	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) 
	{
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		// 在特殊情况下（即无人机朝向与期望完全相反时），将简化四元数 qd_red 重新设为完整的期望姿态 qd。这是为了避免数值不稳定的极端情况。
		qd_red = qd;
	} 
	else 
	{
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		//如果没有发生极端情况，将简化姿态与当前姿态相乘，生成修正后的简化姿态 qd_red。这个操作将当前姿态和期望姿态混合，使姿态更加平滑。
		qd_red *= q;
	}

	// mix full and reduced desired attitude
	// 计算简化姿态 qd_red 与完整姿态 qd 之间的四元数差异 q_mix。该四元数描述了姿态的差异，用于后续的姿态修正和插值。
	Quatf q_mix = qd_red.inversed() * qd;
	// 对四元数 q_mix 进行规范化处理，确保它的数值在有效范围内，消除数值计算中的潜在问题。
	q_mix.canonicalize();
	// catch numerical problems with the domain of acosf and asinf
	// 将四元数的实部 q_mix(0) 限制在 [-1, 1] 之间，以防止由于数值误差导致的超出有效范围。
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);
	// 同样限制四元数的虚部 q_mix(3)，以确保它处于 [-1, 1] 的有效范围内。
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	/* 通过插值的方法混合简化的姿态 qd_red 和完整的期望姿态 qd，使用偏航权重 _yaw_w 进行插值计算。
	cosf(_yaw_w * acosf(q_mix(0))) 计算插值的角度。
	Quatf(..., ..., ..., ...) 创建一个四元数表示插值后的姿态。 */
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));

	// quaternion attitude control law, qe is rotation from q to qd
	// 计算当前姿态与最终期望姿态之间的姿态误差四元数 qe。该四元数表示从当前姿态旋转到期望姿态的旋转量。
	const Quatf qe = q.inversed() * qd;

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	// 将四元数误差 qe 转换为姿态误差向量 eq，该向量表示旋转轴上的误差。
	// 2.f 作为比例常数，表示将四元数误差转换为实际的角度误差。
	const Vector3f eq = 2.f * qe.canonical().imag();

	_attitude_error.attitude_error_roll = eq(0);
	_attitude_error.attitude_error_pitch = eq(1);
	_attitude_error.attitude_error_yaw = eq(2);

	_attitude_error.timestamp = hrt_absolute_time();
	_robust_control_attitude_error_pub.publish(_attitude_error);

	// calculate angular rates setpoint
	// 将姿态误差向量 eq 乘以比例增益 _proportional_gain，生成角速度设定值 rate_setpoint。
	// 这是基于比例控制律的姿态控制输出。
	matrix::Vector3f rate_setpoint = eq.emult(_proportional_gain);

	// Feed forward the yaw setpoint rate.
	// yawspeed_setpoint is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yawspeed_setpoint).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	/* 如果偏航设定速度 _yawspeed_setpoint 是有限的（非 NaN），则将它作为前馈项添加到角速度设定值中。
	q.inversed().dcm_z() 提取当前姿态中世界坐标系下的 Z 轴，并将其与偏航设定速度相乘，表示期望绕 Z 轴的旋转速度。 */
	if (is_finite(_yawspeed_setpoint)) 
	{
		rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
	}

	// limit rates
	/* 将生成的角速度设定值 rate_setpoint 限制在允许的范围内，防止输出角速度过大。
	math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i)) 限制每个轴上的角速度在最大允许值 _rate_limit(i) 范围内。 */
	for (int i = 0; i < 3; i++) 
	{
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
