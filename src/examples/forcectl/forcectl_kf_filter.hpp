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
 * @file frocectl_kf_filter.hpp
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */
// 避免重复包含头文件的编译指令。这确保了在整个项目中只包含一次该文件。
// #pragma once

// #include <matrix/math.hpp>
// #include <mathlib/mathlib.h>
// #include <matrix/Matrix.hpp>
// #include <matrix/Vector.hpp>
// #include <uORB/uORB.h>
// #include <uORB/topics/forcectl_kf_filterdata.h>

// // 定义一个宏，控制是否启用 forcectl_kf_filterdata 数据的存储和发布，0 表示禁用。
// #define SAVE_FORCECTL_KF_FILTER_DATA 0

// class ForcectlKfFilter
// {
// public:
// 	// 初始化 _force 状态向量的第一个和第二个元素为 0.0
// 	// 初始化 _covariance 协方差矩阵的对角线为 10.0，表示初始不确定性较高
// 	ForcectlKfFilter() 
// 	{
// 		_force(0) = 0.0f;
// 		_force(1) = 0.0f;
// 		_covariance(0,0) = 10.0f;
// 		_covariance(1,1) = 10.0f;
// 	}

// 	/**
// 	 * Constructor, initialize state
// 	 */
// 	// 带参数的构造函数声明，可用于设定初始协方差
// 	ForcectlKfFilter(float covInit);

// 	~ForcectlKfFilter() {}

// 	// 定义 force_kf_filter 函数，计算卡尔曼滤波的输出。dt 是时间增量，inputdata 是输入数据
// 	float force_kf_filter(float dt, float inputdata);

// 	/**
// 	 * Get the current filter state
// 	 * @param state0 First state
// 	 * @param state1 Second state
// 	 */
// 	// getState 函数用于获取当前的滤波状态，返回两个状态变量 state0 和 state1
// 	void getState(float &state0, float &state1);

// 	/**
// 	 * Get state variances (diagonal elements)
// 	 * @param cov00 Variance of first state
// 	 * @param cov11 Variance of second state
// 	 */
// 	// getCovariance 函数用于获取状态的方差，返回协方差矩阵的对角元素 cov00 和 cov11
// 	void getCovariance(float &cov00, float &cov11);

// 	/**
// 	 * Get measurement innovation and covariance of last update call
// 	 * @param innov Measurement innovation
// 	 * @param innovCov Measurement innovation covariance
// 	 */
// 	// getInnovations 函数返回最新测量更新的偏差和协方差，用于评估滤波器效果
// 	void getInnovations(float &innov, float &innovCov);

// private:
// 	/**
// 	 * Predict the state with an external acceleration estimate
// 	 * @param dt            Time delta in seconds since last state change
// 	 * @param acc           Acceleration estimate
// 	 * @param acc_unc       Variance of acceleration estimate
// 	 */
// 	// predict 函数用于预测状态，参数包括时间增量 dt、加速度估计 acc 和加速度的不确定性 acc_unc
// 	void predict(float dt, float acc, float acc_unc);

// 	/**
// 	 * Update the state estimate with a measurement
// 	 * @param meas    state measeasurement
// 	 * @param measUnc measurement uncertainty
// 	 * @return update success (measurement not rejected)
// 	 */
// 	// update 函数用于更新状态，meas 是测量值，measUnc 是测量的不确定性，返回布尔值表示更新是否成功
// 	bool update(float meas, float measUnc);

// 	// _force 为状态向量，包含力传感器的两个状态
// 	matrix::Vector<float, 2> _force; // state

// 	// _covariance 为协方差矩阵，表示状态不确定性
// 	matrix::Matrix<float, 2, 2> _covariance; // state covariance

// 	// _cov_measure 表示测量噪声的协方差
// 	float _cov_measure{0.05f}; 

// 	// _cov_estimate 表示估计噪声的协方差
// 	float _cov_estimate{5.0f};

// 	// _residual 表示上一次测量更新的偏差
// 	float _residual{0.0f}; // residual of last measurement update

// 	// _innovCov 表示上一次测量更新的创新协方差
// 	float _innovCov{0.0f}; // innovation covariance of last measurement update

// // 仅在 SAVE_FORCECTL_KF_FILTER_DATA 定义为 1 时有效，声明 forcectl_kf_filterdata_s 
// // 结构用于保存卡尔曼滤波器的数据，并通过 orb_advertise 创建数据发布
// #if SAVE_FORCECTL_KF_FILTER_DATA
// 	forcectl_kf_filterdata_s forcectl_kf_data = {0};
// 	orb_advert_t kf_filter_data_pub = orb_advertise(ORB_ID(forcectl_kf_filterdata), &forcectl_kf_data);
// #endif
// };

#pragma once

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/forcectl_kf_filterdata.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

#define SAVE_FORCECTL_KF_FILTER_DATA 1

class ForcectlKfFilter : public ModuleBase<ForcectlKfFilter>, public ModuleParams
{
public:
	ForcectlKfFilter() : ModuleParams(nullptr){
		_force(0) = 0.0f;
		_force(1) = 0.0f;
		_covariance(0,0) = 10.0f;
		_covariance(1,1) = 10.0f;
	}

	/**
	 * Constructor, initialize state
	 */
	ForcectlKfFilter(float covInit);

	~ForcectlKfFilter() override {}

	float force_kf_filter(float dt, float inputdata);

	/**
	 * Get the current filter state
	 * @param state0 First state
	 * @param state1 Second state
	 */
	void getState(float &state0, float &state1);

	/**
	 * Get state variances (diagonal elements)
	 * @param cov00 Variance of first state
	 * @param cov11 Variance of second state
	 */
	void getCovariance(float &cov00, float &cov11);

	/**
	 * Get measurement innovation and covariance of last update call
	 * @param innov Measurement innovation
	 * @param innovCov Measurement innovation covariance
	 */
	void getInnovations(float &innov, float &innovCov);

private:
	/**
	 * Predict the state with an external acceleration estimate
	 * @param dt            Time delta in seconds since last state change
	 * @param acc           Acceleration estimate
	 * @param acc_unc       Variance of acceleration estimate
	 */
	void predict(float dt, float acc, float acc_unc);

	/**
	 * Update the state estimate with a measurement
	 * @param meas    state measeasurement
	 * @param measUnc measurement uncertainty
	 * @return update success (measurement not rejected)
	 */
	bool update(float meas, float measUnc);

	matrix::Vector<float, 2> _force; // state

	matrix::Matrix<float, 2, 2> _covariance; // state covariance

	float _cov_measure{0.05f};

	float _cov_estimate{5.0f};

	float _residual{0.0f}; // residual of last measurement update

	float _innovCov{0.0f}; // innovation covariance of last measurement update

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void	parameters_update();

 	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FORCECTL_SD_ES>) _param_forcekf_sd_estimate,
		(ParamFloat<px4::params::FORCECTL_SD_ME>) _param_forcekf_sd_measure
	)

	uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};

#if SAVE_FORCECTL_KF_FILTER_DATA
	forcectl_kf_filterdata_s forcectl_kf_data = {0};
	orb_advert_t kf_filter_data_pub = orb_advertise(ORB_ID(forcectl_kf_filterdata), &forcectl_kf_data);
#endif
};
