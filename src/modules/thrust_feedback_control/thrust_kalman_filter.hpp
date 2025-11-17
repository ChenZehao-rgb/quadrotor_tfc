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

#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include <matrix/Vector.hpp>
#include <uORB/uORB.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

class ThrustKalmanFilter : public ModuleBase<ThrustKalmanFilter>, public ModuleParams
{
public:
    ThrustKalmanFilter() : ModuleParams(nullptr)
    {
		_thrust_BFS1(0) = 0.0f;
		_thrust_BFS1(1) = 0.0f;
		_covariance_BFS1(0,0) = 10.0f;
		_covariance_BFS1(1,1) = 10.0f;
        _thrust_BFS2(0) = 0.0f;
		_thrust_BFS2(1) = 0.0f;
		_covariance_BFS2(0,0) = 10.0f;
		_covariance_BFS2(1,1) = 10.0f;
        _thrust_BFS3(0) = 0.0f;
		_thrust_BFS3(1) = 0.0f;
		_covariance_BFS3(0,0) = 10.0f;
		_covariance_BFS3(1,1) = 10.0f;
        _thrust_BFS4(0) = 0.0f;
		_thrust_BFS4(1) = 0.0f;
		_covariance_BFS4(0,0) = 10.0f;
		_covariance_BFS4(1,1) = 10.0f;
	}

    ThrustKalmanFilter(float covInit);

    ~ThrustKalmanFilter() override {}

    float thrust_kalman_filter_BFS1(float dt, float inputdata);
    float thrust_kalman_filter_BFS2(float dt, float inputdata);
    float thrust_kalman_filter_BFS3(float dt, float inputdata);
    float thrust_kalman_filter_BFS4(float dt, float inputdata);

    // void getState(float &state0, float &state1);

    // void getCovariance(float &cov00, float &cov11);

    // void getInnovations(float &innov, float &innovCov);

    // 一阶卡尔曼
    float BFS1_forcekalman_t;
    float BFS1_PKalman_t;
    float BFS1_forcePredictive_t, BFS1_PPredictive_t, BFS1_Kg_t;
    float BFS1_One_Order_Kalman(float inputdata);

    // u_1一阶卡尔曼
    float U1_forcekalman_t;
    float U1_PKalman_t;
    float U1_PPredictive_t, U1_Kg_t;
    float thrust_kalman_filter_u1(float mea, float pre);
    // u_2一阶卡尔曼
    float U2_forcekalman_t;
    float U2_PKalman_t;
    float U2_PPredictive_t, U2_Kg_t;
    float thrust_kalman_filter_u2(float mea, float pre);
    // u_3一阶卡尔曼
    float U3_forcekalman_t;
    float U3_PKalman_t;
    float U3_PPredictive_t, U3_Kg_t;
    float thrust_kalman_filter_u3(float mea, float pre);
    // u_4一阶卡尔曼
    float U4_forcekalman_t;
    float U4_PKalman_t;
    float U4_PPredictive_t, U4_Kg_t;
    float thrust_kalman_filter_u4(float mea, float pre);

private:
    // R
    float _cov_measure{0.05f};
    // Q
    float _cov_estimate{5.0f};

    // BFS_One
    void predict_BFS1(float dt, float acc, float acc_unc);
    bool update_BFS1(float meas, float measUnc);
    matrix::Vector<float, 2> _thrust_BFS1;
    matrix::Matrix<float, 2, 2> _covariance_BFS1;
    float _residual_BFS1{0.0f};
    float _innovCov_BFS1{0.0f};
    // BFS_Two
    void predict_BFS2(float dt, float acc, float acc_unc);
    bool update_BFS2(float meas, float measUnc);
    matrix::Vector<float, 2> _thrust_BFS2;
    matrix::Matrix<float, 2, 2> _covariance_BFS2;
    float _residual_BFS2{0.0f};
    float _innovCov_BFS2{0.0f};
    // BFS_Three
    void predict_BFS3(float dt, float acc, float acc_unc);
    bool update_BFS3(float meas, float measUnc);
    matrix::Vector<float, 2> _thrust_BFS3;
    matrix::Matrix<float, 2, 2> _covariance_BFS3;
    float _residual_BFS3{0.0f};
    float _innovCov_BFS3{0.0f};
    // BFS_Four
    void predict_BFS4(float dt, float acc, float acc_unc);
    bool update_BFS4(float meas, float measUnc);
    matrix::Vector<float, 2> _thrust_BFS4;
    matrix::Matrix<float, 2, 2> _covariance_BFS4;
    float _residual_BFS4{0.0f};
    float _innovCov_BFS4{0.0f};
    
    // 一阶卡尔曼
    float BFS1_forcekalman_t_1 = 0.0f;
    float BFS1_Pkalman_t_1 = 2.0f;
    float BFS1_Q = 0.005f;
    float BFS1_R = 0.36f;

    // u_1
    float _cov_estimate_u{0.36f};
    float _cov_measure_u{0.005f};
    float U1_Pkalman_t_1 = 2.0f;
    float U2_Pkalman_t_1 = 2.0f;
    float U3_Pkalman_t_1 = 2.0f;
    float U4_Pkalman_t_1 = 2.0f;

    /**
	 * initialize some vectors/matrices from parameters
	 */
	void	parameters_update();
    DEFINE_PARAMETERS(
       (ParamFloat<px4::params::TFC_SD_ES>) _param_sd_estimate,
       (ParamFloat<px4::params::TFC_SD_ME>) _param_sd_measure,
       (ParamFloat<px4::params::TFC_ME_U>) _param_measure_u,
       (ParamFloat<px4::params::TFC_ES_U>) _param_estimate_u
    )
    uORB::SubscriptionInterval	_parameter_update_sub{ORB_ID(parameter_update), 1_s};
};

