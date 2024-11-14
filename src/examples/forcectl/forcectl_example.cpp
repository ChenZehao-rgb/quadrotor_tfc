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

#include "forcectl_example.hpp"

// appState 是一个 px4::AppState 对象，用于管理应用程序的状态（如启动、停止等）。
px4::AppState Forcectl::appState;
// 定义了两个二阶低通滤波器对象，分别用于对 p_out 和 d_out 的 PID 输出进行滤波，减少高频噪声。
// 100.f 是采样频率，30.f 和 10.f 是滤波器的截止频率。
math::LowPassFilter2p<float>	_pid_pout_lowpass_filter{100.f, 30.f};
math::LowPassFilter2p<float>	_pid_dout_lowpass_filter{100.f, 10.f};

// 定义了一个增量式 PID 控制器的结构体。
// 包括 PID 参数（Kp、Ki、Kd）、输出值（p_out、i_out、d_out）、误差值以及输出结果。
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float p_out;
	float i_out;
	float d_out;
	float err;
	float last_err;
	float previous_err;
	float output;
}incremental_PID;

// 定义了一个 ADRC （自抗扰控制器）的结构体，
// 用于存储控制器的相关状态变量和参数，包括 TD_v1（跟踪微分器）、ESO_z1（扩展状态观测器）等。
typedef struct
{
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

// 增量式PID计算
void Incremental_PID_calculate(incremental_PID *pid)
{
	// 检查 pid 是否为空指针，以防止空指针引用。
	if(pid == NULL)
	{
		return;
	}

	// 计算 p_out，即比例输出。它基于当前误差 err 和上一次的误差 last_err。
	pid->p_out = pid->Kp * (pid->err - pid->last_err);
	// 计算积分输出 i_out，基于当前的误差 err。
	pid->i_out = pid->Ki * pid->err;
	// 计算微分输出 d_out，基于当前误差、上一次误差和上上次误差的变化。
	pid->d_out = pid->Kd * (pid->err - 2.0f*pid->last_err + pid->previous_err);

	// 对 p_out 和 d_out 进行滤波，以消除噪声。
	pid->p_out = _pid_pout_lowpass_filter.apply(pid->p_out);
	pid->d_out = _pid_dout_lowpass_filter.apply(pid->d_out);

	// 将比例、积分和微分的结果加到输出值中。
	pid->output += pid->p_out + pid->i_out + pid->d_out;
	// 将输出限制在 [0, 1] 的范围内，防止输出过大或过小。
	pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);

	// 更新误差值，准备下一次的 PID 计算。
	pid->previous_err = pid->last_err;
	pid->last_err = pid->err;
}

// ADRC相关函数
// 该函数返回输入值 in1 的符号。如果 in1 大于 0，返回 1；如果小于 0，返回 -1；如果接近 0，返回 0。
float ADRC_sign(float in1)
{
	return ((in1 > 0.0f) ? 1.0f : ((fabs(in1 - 0.0f) < 1e-7) ? 0.0f : -1.0f));
}
// 该函数用于计算自抗扰控制器的符号函数的一个特殊形式，
// 返回 (sign(in1 + in2) - sign(in1 - in2)) / 2.0f，用于特定计算场合。
float ADRC_fsg(float in1, float in2)
{
	return (ADRC_sign(in1 + in2) - ADRC_sign(in1 - in2))/2.0f;
}
// 这是一个跟踪微分器函数，用于计算跟踪误差的动态变化，
// 它基于 in1 和 in2 的输入误差，以及控制参数 in3 和 in4。
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
// 该函数是自抗扰控制器的 fal 函数，用于估计误差的非线性函数。
// 它根据输入误差 in1，调整灵敏度参数 in2 和范围 in3。
float ADRC_fal(float in1, float in2, float in3)
{
	float s = (ADRC_sign(in1 + in3) - ADRC_sign(in1 - in3))/2.0f;
	double fal = (double)in1*(double)s/(pow(in3, (1.0f - in2))) + pow(fabs(in1), in2)*(double)ADRC_sign(in1)*(1.0 - (double)s);

	return (float)fal;
}
// 这是自抗扰控制器的主要计算函数。
void ADRC_calculate(ADRC *adrc)
{
	if(adrc == NULL)
	{
		return;
	}

	// 获取当前时间，并计算控制回路的时间步长 dt。
	uint64_t time_now = hrt_absolute_time();
	float dt = (float)(time_now - adrc->last_timestamp)/1000000.0f;
	// 将 dt 限制在合理范围内，防止时间步长过大或过小。
	dt = (dt < 0.001f) ? 0.02f : ((dt > 0.04f) ? 0.02f : dt);
	adrc->dt = dt;
	adrc->last_timestamp = time_now;

	/*ADRC-TD*/
	// 计算跟踪误差 e_TD 并通过 fhan 函数更新跟踪微分器的状态 TD_v1 和 TD_v2。
	float e_TD = adrc->TD_v1 - adrc->force_exp;
	float fh = ADRC_fhan(e_TD, adrc->TD_v2, 400.0f, dt);
	adrc->TD_v1 = adrc->TD_v1 + adrc->TD_v2*dt;
	adrc->TD_v2 = adrc->TD_v2 + fh*dt;

	/*CONTROLLER*/
	// 计算控制器的误差 e1_CTL 和 e2_CTL，并根据 Kp 和 Kd 更新控制信号 u_CTL。
	float e1_CTL = adrc->TD_v1 - adrc->ESO_z1;
	float e2_CTL = adrc->TD_v2 - adrc->ESO_z2;
	adrc->u0_CTL += adrc->Kp*e1_CTL + adrc->Kd*e2_CTL;
	float u_CTL = (adrc->u0_CTL - adrc->ESO_z3)/adrc->b0;

	/*ESO*/
	// 通过扩展状态观测器（ESO）对控制器的误差 e_ESO 进行估计，并更新状态变量 ESO_z1, ESO_z2, ESO_z3。
	float e_ESO = adrc->ESO_z1 - adrc->force_mea;
	float fe = ADRC_fal(e_ESO, 0.5f, dt);
	float fe1 = ADRC_fal(e_ESO, 0.25f, dt);
	adrc->ESO_z1 = adrc->ESO_z1 + dt*(adrc->ESO_z2 - adrc->beta1*e_ESO);
	adrc->ESO_z2 = adrc->ESO_z2 + dt*(adrc->ESO_z3 - adrc->beta2*fe + adrc->b0*u_CTL);
	adrc->ESO_z3 = adrc->ESO_z3 + dt*(-adrc->beta3*fe1);
	// adrc->ESO_z1 = adrc->ESO_z1 + dt*(adrc->ESO_z2 - adrc->beta1*e_ESO + adrc->b0*u_CTL);
	// adrc->ESO_z2 = adrc->ESO_z2 + dt*(-adrc->beta2*fe);

	/*OUTPUT*/
	// 将控制器的输出 u_CTL 限制在 [0, 0.5] 范围内，确保输出值合理。
	adrc->output = (u_CTL > 0.5f) ? 0.5f : ((u_CTL < 0.0f) ? 0.0f : u_CTL);
}

int Forcectl::main()
{
	// 设置应用程序状态为运行中。
	appState.setRunning(true);

	/* subscribe to forcectl_forcedata topic */
	// 这是力传感器数据的订阅话题，用于获取传感器测得的实时力值。
	int forcedata_sub_fd = orb_subscribe(ORB_ID(forcectl_forcedata));
	/* limit the update rate to 50 Hz */
	orb_set_interval(forcedata_sub_fd, 20);

	/* subscribe to actuator_controls_3 topic */
	// 这个话题包含的是通过遥控器设置的期望力输出。
	int forceexp_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3));
	/* limit the update rate to 50 Hz */
	orb_set_interval(forceexp_sub_fd, 20);

	/* subscribe to rc_channels topic */
	// 这个话题订阅的是遥控器各个通道的数据（例如：PID控制器的参数调整或控制模式切换）。
	int pid_sub_fd = orb_subscribe(ORB_ID(rc_channels));
	/* limit the update rate to 50 Hz */
	orb_set_interval(pid_sub_fd, 20);

	/* advertise forcectl_controldata topic */
	// 初始化控制数据：control_data 是用于发布控制信号的数据结构，初始化为 0。
	// 发布话题：forcectl_controldata 是用来发布控制数据的自定义话题，orb_advertise 函数用来创建这个发布话题。
	struct forcectl_controldata_s control_data;
	memset(&control_data, 0, sizeof(control_data));
	orb_advert_t controldata_pub = orb_advertise(ORB_ID(forcectl_controldata), &control_data);

	// 创建轮询数组：这个数组包含两个文件描述符，用于监听 forcectl_forcedata 和 actuator_controls_3 话题是否有新数据到达。
	px4_pollfd_struct_t fds[] = 
	{
		{ .fd = forcedata_sub_fd,   .events = POLLIN },
		{ .fd = forceexp_sub_fd,   .events = POLLIN },
	};

	// 赋初值 0
	struct forcectl_forcedata_s forcedata = {};
	struct actuator_controls_s force_exp_from_rc = {};
	struct rc_channels_s rc_channals_data = {};
	static incremental_PID incre_pid = {};
	static ADRC adrc = {};

	_pid_pout_lowpass_filter.reset(0.0);
	_pid_dout_lowpass_filter.reset(0.0);

	while(appState.isRunning())
	{
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		// 使用 px4_poll 函数等待最多 1 秒（1000ms），如果其中一个话题有数据更新，则会处理数据。
		int poll_ret = px4_poll(fds, 2, 1000);

		/* handle the poll result */
		if (poll_ret == 0) 
		{
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} 
		else if (poll_ret > 0) 
		{

			// 处理力传感器数据：当 forcectl_forcedata 话题有更新时，将新数据拷贝到 forcedata 结构体。
			if (fds[0].revents & POLLIN) 
			{
				orb_copy(ORB_ID(forcectl_forcedata), forcedata_sub_fd, &forcedata);
			}

			// 处理期望力数据：从 actuator_controls_3 中读取期望力，
			// 并通过计算将其与传感器数据结合起来，存储在 control_data.force_exp 中。
			if (fds[1].revents & POLLIN) 
			{
				orb_copy(ORB_ID(actuator_controls_3), forceexp_sub_fd, &force_exp_from_rc);
				control_data.force_exp = forcedata.force_max*force_exp_from_rc.control[3];
			}
		}

		// 控制模式切换
		// 从 rc_channels 话题读取遥控器通道数据，判断不同的控制模式并执行相应的控制逻辑。
		// PID复位：当 channels[4] < 0 或 channels[5] 超过某个范围时，复位 PID 和 ADRC 参数。
		orb_copy(ORB_ID(rc_channels), pid_sub_fd, &rc_channals_data);
		if((rc_channals_data.channels[4] < 0.0f)||(rc_channals_data.channels[5] < -0.3f)||(rc_channals_data.channels[5] > 0.3f))
		{
			memset(&incre_pid, 0, sizeof(incre_pid));
		}
		if((rc_channals_data.channels[4] < 0.0f)||(rc_channals_data.channels[5] < 0.3f))
		{
			memset(&adrc, 0, sizeof(adrc));
		}

		// 增量 PID 控制：当 channels[5] 在一定范围内时，执行增量 PID 控制。
		if((rc_channals_data.channels[5] > -0.3f)&&(rc_channals_data.channels[5] < 0.3f))
		{
			control_data.kp = 2.0f*(rc_channals_data.channels[6] + 1.0f)/2.0f;
			control_data.ki = 0.2f*(rc_channals_data.channels[7] + 1.0f)/2.0f;
			control_data.kd = 5.0f*(rc_channals_data.channels[8] + 1.0f)/2.0f;

			control_data.force_error = control_data.force_exp - forcedata.force_kf_filtered_data;
			incre_pid.Kp = control_data.kp;
			incre_pid.Ki = control_data.ki;
			incre_pid.Kd = control_data.kd;
			incre_pid.err = control_data.force_error;

			Incremental_PID_calculate(&incre_pid);

			control_data.force_control_out = incre_pid.output;
		}
		// ADRC 控制：当 channels[5] > 0.3f 时，执行自抗扰控制器（ADRC）。
		else if(rc_channals_data.channels[5] > 0.3f)
		{
			adrc.b0 = 60000.0f;//(float)0.028e4/(1.292f*0.055f);
			adrc.beta1 = 50.0f;
			adrc.beta2 = 100.0f;
			adrc.beta3 = 200.0f;
			adrc.Kp = 0.08f*(rc_channals_data.channels[6] + 1.0f)/2.0f;
			adrc.Kd = 0.01f*(rc_channals_data.channels[7] + 1.0f)/2.0f;
			adrc.force_exp = control_data.force_exp;
			adrc.force_mea = forcedata.force_kf_filtered_data;

			if(rc_channals_data.channels[4] > 0.0f)
			{
				ADRC_calculate(&adrc);
			}

			control_data.force_control_out = adrc.output;
		}
		// 停止控制输出：当 channels[5] < -0.3f 时，停止力控制输出。
		else if(rc_channals_data.channels[5] < -0.3f)
		{
			control_data.force_control_out = 0.0f;
		}

// 记录ADRC数据（可选）：通过宏 SAVE_FORCECTL_ADRC_DATA，可以选择性记录 ADRC 的内部状态变量，如 TD_v1、ESO_z1 等。
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

		// 发布控制数据：每个周期结束时，发布 forcectl_controldata。
		forcectl_ADRC_data.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(forcectl_adrc_data), adrc_data_pub, &forcectl_ADRC_data);
#endif

		control_data.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(forcectl_controldata), controldata_pub, &control_data);

		px4_usleep(10000);
	}

	return 0;
}
