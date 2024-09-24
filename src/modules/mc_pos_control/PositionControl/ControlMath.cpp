/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ControlMath
{

/* 将推力向量（thr_sp）转换为一个姿态设定（att_sp）的函数。
它结合了推力方向与大小，以及期望的航向（yaw_sp），来计算无人机的姿态设定值。 */
void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	/* 将推力向量和航向设定值转换为姿态设定。
	-thr_sp 代表推力方向的负值，因为推力通常是在机体坐标系的负 Z 轴方向上作用的。
	通过这个函数可以将推力向量方向和大小转换为 UAV 的期望姿态，并将结果保存到 att_sp 中。
	yaw_sp：这个参数代表期望的航向角，即无人机绕 Z 轴的旋转方向。 */
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	/* thr_sp.length() 计算的是推力向量的模，即推力的大小。
	推力大小存储在姿态设定的推力 Z 轴分量中，因为推力通常作用在无人机的 Z 轴上。
	-thr_sp.length() 这里的负号是因为在无人机的惯性系中，推力方向通常与 Z 轴相反（向下为正 Z 轴，
	推力方向为负 Z 轴），所以需要用负号来修正方向。 */
	att_sp.thrust_body[2] = -thr_sp.length();
}

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	// determine tilt
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);
	// limit tilt
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// corner case exactly parallel vectors
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

/* 根据期望的推力方向向量（body_z）和航向角（yaw_sp），计算出无人机的姿态，
并将姿态结果填充到 att_sp 结构体中。
该姿态包括无人机的姿态矩阵（旋转矩阵）、四元数表示的姿态以及欧拉角表示的姿态。 */
void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// zero vector, no direction, set safe level value
	/* 检查向量是否为零：body_z.norm_squared() 计算向量 body_z 的平方模，如果其值小于非常小的阈值 FLT_EPSILON（表示几乎为零），
	说明推力方向向量是一个零向量。 */
	if (body_z.norm_squared() < FLT_EPSILON) 
	{
		/* 处理零向量：如果 body_z 是零向量，将 body_z 的 Z 轴分量设为 1。
		这确保即使输入推力向量无效，代码仍然能给出一个合理的向上方向。 */
		body_z(2) = 1.f;
	}

	/* 标准化向量：将推力方向向量 body_z 归一化为单位向量，确保它的长度为 1，用于接下来的姿态计算。 */
	body_z.normalize();

	// vector of desired yaw direction in XY plane, rotated by PI/2
	/* 计算 Y 轴方向向量：这是根据航向角 yaw_sp 计算得到的一个 Y 轴方向向量，
	它位于 XY 平面内，并绕 Z 轴旋转了 90 度（PI/2），用于确保机体的 X 轴保持与推力方向正交。 */
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// desired body_x axis, orthogonal to body_z
	/* 计算机体 X 轴方向：这里通过叉积（% 表示向量叉积）计算出机体的 X 轴方向，使其与推力方向 body_z 正交。 */
	Vector3f body_x = y_C % body_z;

	// keep nose to front while inverted upside down
	/* 保持机头朝前：如果无人机倒飞（推力方向 body_z 的 Z 分量为负值），则将 X 轴方向取反，以确保无人机的 "鼻子" 朝前，即机体朝正确的方向。 */
	if (body_z(2) < 0.0f) 
	{
		body_x = -body_x;
	}

	/* 处理特殊情况：如果推力方向接近完全水平（body_z(2) 几乎为零），则将 X 轴设置为 Z 方向，以确保旋转矩阵可以正确构建。此时，航向角不再重要。 */
	if (fabsf(body_z(2)) < 0.000001f) 
	{
		// desired thrust is in XY plane, set X downside to construct correct matrix,
		// but yaw component will not be used actually
		body_x.zero();
		body_x(2) = 1.0f;
	}

	// 标准化 X 轴向量：确保 X 轴向量为单位向量。
	body_x.normalize();

	// desired body_y axis
	// 计算机体 Y 轴方向：通过叉积计算机体的 Y 轴方向，确保其与 X 轴和 Z 轴均正交。
	const Vector3f body_y = body_z % body_x;

	// 创建旋转矩阵对象：R_sp 用于存储姿态的旋转矩阵。
	Dcmf R_sp;

	// fill rotation matrix
	// 填充旋转矩阵：将计算好的 X、Y、Z 轴方向向量分别填充到旋转矩阵的列中，形成最终的机体姿态矩阵。
	for (int i = 0; i < 3; i++) 
	{
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	// 计算并保存四元数表示的姿态：将旋转矩阵 R_sp 转换为四元数 q_sp，
	// 并将其拷贝到姿态设定 att_sp.q_d 中。四元数用于高效地表示旋转。
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	/* 计算并保存欧拉角表示的姿态：将旋转矩阵 R_sp 转换为欧拉角 euler，
	并将其对应的滚转（roll_body）、俯仰（pitch_body）和偏航（yaw_body）角度存储到 att_sp 中。
	注意，这里的欧拉角仅用于日志记录，而不是用于控制。 */
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();
}

Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		// vector does not exceed maximum magnitude
		return v0 + v1;

	} else if (v0.length() >= max) {
		// the magnitude along v0, which has priority, already exceeds maximum.
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// the two vectors are equal
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		// the first vector is 0.
		return v1.normalized() * max;

	} else {
		// vf = final vector with ||vf|| <= max
		// s = scaling factor
		// u1 = unit of v1
		// vf = v0 + v1 = v0 + s * u1
		// constraint: ||vf|| <= max
		//
		// solve for s: ||vf|| = ||v0 + s * u1|| <= max
		//
		// Derivation:
		// For simplicity, replace v0 -> v, u1 -> u
		// 				   		   v0(0/1/2) -> v0/1/2
		// 				   		   u1(0/1/2) -> u0/1/2
		//
		// ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
		// v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
		// s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
		//
		// quadratic equation:
		// -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
		//
		// b = 2 * u.dot(v)
		// a = 1 (because u is normalized)
		// c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
		//
		// sqrt(b^2 - 4*a*c) =
		// 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
		//
		// s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
		//      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
		// m = u.dot(v)
		// s = -m + sqrt(m^2 - c)
		//
		//
		//
		// notes:
		// 	- s (=scaling factor) needs to be positive
		// 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement
		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	// project center of sphere on line  normalized AB
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		// we have triangle CDX with known CD and CX = R, find DX
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			// target waypoint is already behind us
			res = line_b;

		} else {
			// target is in front of us
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {

		// have no roots, return D
		res = d; // go directly to line

		// previous waypoint is still in front of us
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		// target waypoint is already behind us
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		return false;
	}
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f());
}

} // ControlMath
