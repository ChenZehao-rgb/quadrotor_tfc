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
 * @file Takeoff.cpp
 */

#include "Takeoff.hpp"
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

// 这段代码负责管理无人机起飞的状态机和起飞时的速度控制 

// 该函数生成初始的垂直速度值，用于起飞时的加速斜坡。
void Takeoff::generateInitialRampValue(float velocity_p_gain)
{
	// 速度控制器的比例增益不低于 0.01。比例增益太小会导致响应过慢或不稳定。
	velocity_p_gain = math::max(velocity_p_gain, 0.01f);
	// 根据比例增益计算初始垂直速度限制值。CONSTANTS_ONE_G 是重力加速度常量，单位是 m/s²。
	// 通过除以 velocity_p_gain，这个公式给出一个合理的初始速度值，使得起飞时速度不会立即跃升，而是平滑加速。
	_takeoff_ramp_vz_init = -CONSTANTS_ONE_G / velocity_p_gain;
}

// 该函数根据无人机的状态（是否解锁、是否起飞等）更新起飞状态机。
void Takeoff::updateTakeoffState(const bool armed, const bool landed, const bool want_takeoff,
				 const float takeoff_desired_vz, const bool skip_takeoff, const hrt_abstime &now_us)
{
	/* 这是一个时间延迟逻辑，确保解锁（armed）后，电机有时间加速到稳定转速。now_us 是当前时间，单位是微秒。 */
	_spoolup_time_hysteresis.set_state_and_update(armed, now_us);

	// 状态机逻辑：
	// 根据当前的起飞状态 _takeoff_state 进行不同的操作。
	switch (_takeoff_state) 
	{
	case TakeoffState::disarmed:
	// 如果无人机解锁（armed），进入 spoolup 状态，表示电机开始转动。
		if (armed) 
		{
			_takeoff_state = TakeoffState::spoolup;
		} 
		else 
		{
			break;
		}

	// FALLTHROUGH
	case TakeoffState::spoolup:
	// 在 spoolup 状态下，如果电机已经达到稳定转速，则状态切换到 ready_for_takeoff，表示可以准备起飞。
		if (_spoolup_time_hysteresis.get_state()) 
		{
			_takeoff_state = TakeoffState::ready_for_takeoff;
		} 
		else 
		{
			break;
		}

	// FALLTHROUGH
	case TakeoffState::ready_for_takeoff:
	// 如果用户指令请求起飞（want_takeoff 为真），状态切换到 rampup，并初始化起飞斜坡的进度 _takeoff_ramp_progress 为 0。
		if (want_takeoff) 
		{
			_takeoff_state = TakeoffState::rampup;
			_takeoff_ramp_progress = 0.f;
		} 
		else 
		{
			break;
		}

	// FALLTHROUGH
	case TakeoffState::rampup:
	// 当起飞斜坡进度完成（_takeoff_ramp_progress >= 1.f），状态切换到 flight，表示起飞完成，进入正常飞行状态。
		if (_takeoff_ramp_progress >= 1.f) 
		{
			_takeoff_state = TakeoffState::flight;
		} 
		else 
		{
			break;
		}

	// FALLTHROUGH
	case TakeoffState::flight:
	// 如果无人机检测到降落（landed 为真），状态返回到 ready_for_takeoff，准备下一次起飞。
		if (landed) 
		{
			_takeoff_state = TakeoffState::ready_for_takeoff;
		}
		break;

	default:
		break;
	}

	// 其他情况处理
	// 如果无人机解锁且跳过起飞阶段（skip_takeoff 为真），直接进入飞行状态 flight。
	if (armed && skip_takeoff) 
	{
		_takeoff_state = TakeoffState::flight;
	}

	// TODO: need to consider free fall here
	// 如果无人机解锁且跳过起飞阶段（skip_takeoff 为真），直接进入飞行状态 flight。
	if (!armed) 
	{
		_takeoff_state = TakeoffState::disarmed;
	}
}

// 该函数用于更新起飞斜坡进度，并返回当前垂直速度限制值。
float Takeoff::updateRamp(const float dt, const float takeoff_desired_vz)
{
	// 初始时，垂直速度限制值设置为目标的期望起飞速度 takeoff_desired_vz。
	float upwards_velocity_limit = takeoff_desired_vz;

	/* 如果当前状态还没进入 rampup，将垂直速度限制值设为初始值 _takeoff_ramp_vz_init，确保起飞时逐渐加速。 */
	if (_takeoff_state < TakeoffState::rampup) 
	{
		upwards_velocity_limit = _takeoff_ramp_vz_init;
	}

	// 如果当前处于 rampup 状态，意味着正在执行加速斜坡，继续更新进度。
	if (_takeoff_state == TakeoffState::rampup) 
	{
		/* 计算并更新起飞斜坡的进度。_takeoff_ramp_progress 是从 0 到 1 的一个比例值。
		如果时间步 dt 小于斜坡持续时间 _takeoff_ramp_time，则根据 dt 更新斜坡进度；
		否则直接设置进度为 1，表示斜坡完成。 */
		if (_takeoff_ramp_time > dt) 
		{
			_takeoff_ramp_progress += dt / _takeoff_ramp_time;
		} 
		else 
		{
			_takeoff_ramp_progress = 1.f;
		}

		// 如果斜坡进度还未完成（_takeoff_ramp_progress < 1.f），根据当前进度计算当前的垂直速度限制值，确保垂直速度逐渐增加。
		if (_takeoff_ramp_progress < 1.f) 
		{
			upwards_velocity_limit = _takeoff_ramp_vz_init + _takeoff_ramp_progress * (takeoff_desired_vz - _takeoff_ramp_vz_init);
		}
	}
	
	// 返回当前的垂直速度限制值，用于限制无人机的最大垂直速度，以保证平滑起飞。
	return upwards_velocity_limit;
}
