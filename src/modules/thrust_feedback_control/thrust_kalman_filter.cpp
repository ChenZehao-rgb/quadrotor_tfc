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

#include "thrust_kalman_filter.hpp"

ThrustKalmanFilter::ThrustKalmanFilter(float covInit) : ModuleParams(nullptr)
{
    _thrust_BFS1(0) = 0.0f;
    _thrust_BFS1(1) = 0.0f;
    _covariance_BFS1(0,0) = covInit;
    _covariance_BFS1(1,1) = covInit;
}

void ThrustKalmanFilter::parameters_update()
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}
}

/*
第一个气压式力传感器的二阶卡尔曼滤波
*/
void ThrustKalmanFilter::predict_BFS1(float dt, float acc, float acc_unc)
{
    _thrust_BFS1(0) += _thrust_BFS1(1) * dt + acc * dt * dt / 2;
    _thrust_BFS1(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_BFS1 = A * _covariance_BFS1 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_BFS1(float meas, float measUnc)
{
    _residual_BFS1 = meas - _thrust_BFS1(0);

    _innovCov_BFS1 = _covariance_BFS1(0, 0) + (measUnc * measUnc);

    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_BFS1(0, 0);
    kalmanGain(1) = _covariance_BFS1(1, 0);
    kalmanGain /= _innovCov_BFS1;

    _thrust_BFS1 += kalmanGain * _residual_BFS1;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_BFS1 = (identity - KH) * _covariance_BFS1;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_BFS1(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_sd_estimate.get();
    _cov_measure = _param_sd_measure.get();

    predict_BFS1(dt, 0.0f, _cov_estimate);
    update_BFS1(inputdata, _cov_measure);

    return _thrust_BFS1(0);
}

/*
第二个气压式力传感器的二阶卡尔曼滤波
*/
void ThrustKalmanFilter::predict_BFS2(float dt, float acc, float acc_unc)
{
    _thrust_BFS2(0) += _thrust_BFS2(1) * dt + acc * dt * dt / 2;
    _thrust_BFS2(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_BFS2 = A * _covariance_BFS2 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_BFS2(float meas, float measUnc)
{
    _residual_BFS2 = meas - _thrust_BFS2(0);

    _innovCov_BFS2 = _covariance_BFS2(0, 0) + (measUnc * measUnc);

    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_BFS2(0, 0);
    kalmanGain(1) = _covariance_BFS2(1, 0);
    kalmanGain /= _innovCov_BFS2;

    _thrust_BFS2 += kalmanGain * _residual_BFS2;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_BFS2 = (identity - KH) * _covariance_BFS2;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_BFS2(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_sd_estimate.get();
    _cov_measure = _param_sd_measure.get();

    predict_BFS2(dt, 0.0f, _cov_estimate);
    update_BFS2(inputdata, _cov_measure);

    return _thrust_BFS2(0);
}

/*
第三个气压式力传感器的二阶卡尔曼滤波
*/
void ThrustKalmanFilter::predict_BFS3(float dt, float acc, float acc_unc)
{
    _thrust_BFS3(0) += _thrust_BFS3(1) * dt + acc * dt * dt / 2;
    _thrust_BFS3(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_BFS3 = A * _covariance_BFS3 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_BFS3(float meas, float measUnc)
{
    _residual_BFS3 = meas - _thrust_BFS3(0);

    _innovCov_BFS3 = _covariance_BFS3(0, 0) + (measUnc * measUnc);

    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_BFS3(0, 0);
    kalmanGain(1) = _covariance_BFS3(1, 0);
    kalmanGain /= _innovCov_BFS3;

    _thrust_BFS3 += kalmanGain * _residual_BFS3;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_BFS3 = (identity - KH) * _covariance_BFS3;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_BFS3(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_sd_estimate.get();
    _cov_measure = _param_sd_measure.get();

    predict_BFS3(dt, 0.0f, _cov_estimate);
    update_BFS3(inputdata, _cov_measure);

    return _thrust_BFS3(0);
}

/*
第四个气压式力传感器的二阶卡尔曼滤波
*/
void ThrustKalmanFilter::predict_BFS4(float dt, float acc, float acc_unc)
{
    _thrust_BFS4(0) += _thrust_BFS4(1) * dt + acc * dt * dt / 2;
    _thrust_BFS4(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_BFS4 = A * _covariance_BFS4 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_BFS4(float meas, float measUnc)
{
    _residual_BFS4 = meas - _thrust_BFS4(0);

    _innovCov_BFS4 = _covariance_BFS4(0, 0) + (measUnc * measUnc);

    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_BFS4(0, 0);
    kalmanGain(1) = _covariance_BFS4(1, 0);
    kalmanGain /= _innovCov_BFS4;

    _thrust_BFS4 += kalmanGain * _residual_BFS4;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_BFS4 = (identity - KH) * _covariance_BFS4;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_BFS4(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_sd_estimate.get();
    _cov_measure = _param_sd_measure.get();

    predict_BFS4(dt, 0.0f, _cov_estimate);
    update_BFS4(inputdata, _cov_measure);

    return _thrust_BFS4(0);
}

////////////////////////////////////////////////////////////////////////
// 期望升力的二阶卡尔曼滤波--求解期望升力的一阶微分
////////////////////////////////////////////////////////////////////////
/*
第一个期望升力的一阶微分预测与更新
*/
void ThrustKalmanFilter::predict_DThrust1(float dt, float acc, float acc_unc)
{
    _thrust_DThrust1(0) += _thrust_DThrust1(1) * dt + acc * dt * dt / 2;
    _thrust_DThrust1(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_DThrust1 = A * _covariance_DThrust1 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_DThrust1(float meas, float measUnc)
{
    _residual_DThrust1 = meas - _thrust_DThrust1(0);

    _innovCov_DThrust1 = _covariance_DThrust1(0, 0) + (measUnc * measUnc);

    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_DThrust1(0, 0);
    kalmanGain(1) = _covariance_DThrust1(1, 0);
    kalmanGain /= _innovCov_DThrust1;

    _thrust_DThrust1 += kalmanGain * _residual_DThrust1;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_DThrust1 = (identity - KH) * _covariance_DThrust1;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_DThrust1(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_td_estimate.get();
    _cov_measure = _param_td_measure.get();

    predict_DThrust1(dt, 0.0f, _cov_estimate);
    update_DThrust1(inputdata, _cov_measure);

    return _thrust_DThrust1(1);
}

/*
第二个期望升力的一阶微分预测与更新
*/
void ThrustKalmanFilter::predict_DThrust2(float dt, float acc, float acc_unc)
{
    _thrust_DThrust2(0) += _thrust_DThrust2(1) * dt + acc * dt * dt / 2;
    _thrust_DThrust2(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_DThrust2 = A * _covariance_DThrust2 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_DThrust2(float meas, float measUnc)
{
    _residual_DThrust2 = meas - _thrust_DThrust2(0);

    _innovCov_DThrust2 = _covariance_DThrust2(0, 0) + (measUnc * measUnc);
    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_DThrust2(0, 0);
    kalmanGain(1) = _covariance_DThrust2(1, 0);
    kalmanGain /= _innovCov_DThrust2;
    _thrust_DThrust2 += kalmanGain * _residual_DThrust2;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_DThrust2 = (identity - KH) * _covariance_DThrust2;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_DThrust2(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_td_estimate.get();
    _cov_measure = _param_td_measure.get();

    predict_DThrust2(dt, 0.0f, _cov_estimate);
    update_DThrust2(inputdata, _cov_measure);

    return _thrust_DThrust2(1);
}

/*
第三个期望升力的一阶微分预测与更新
*/
void ThrustKalmanFilter::predict_DThrust3(float dt, float acc, float acc_unc)
{
    _thrust_DThrust3(0) += _thrust_DThrust3(1) * dt + acc * dt * dt / 2;
    _thrust_DThrust3(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_DThrust3 = A * _covariance_DThrust3 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_DThrust3(float meas, float measUnc)
{
    _residual_DThrust3 = meas - _thrust_DThrust3(0);

    _innovCov_DThrust3 = _covariance_DThrust3(0, 0) + (measUnc * measUnc);
    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_DThrust3(0, 0);
    kalmanGain(1) = _covariance_DThrust3(1, 0);
    kalmanGain /= _innovCov_DThrust3;
    _thrust_DThrust3 += kalmanGain * _residual_DThrust3;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_DThrust3 = (identity - KH) * _covariance_DThrust3;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_DThrust3(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_td_estimate.get();
    _cov_measure = _param_td_measure.get();

    predict_DThrust3(dt, 0.0f, _cov_estimate);
    update_DThrust3(inputdata, _cov_measure);

    return _thrust_DThrust3(1);
}

/*
第四个期望升力的一阶微分预测与更新
*/
void ThrustKalmanFilter::predict_DThrust4(float dt, float acc, float acc_unc)
{
    _thrust_DThrust4(0) += _thrust_DThrust4(1) * dt + acc * dt * dt / 2;
    _thrust_DThrust4(1) += acc * dt;

    matrix::Matrix<float, 2, 2> A;
    A(0, 0) = 1;
    A(1, 1) = 1;
    A(0, 1) = dt;

    matrix::Matrix<float, 2, 1> G;
    G(0, 0) = dt * dt / 2;
    G(1, 0) = dt;

    matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

    _covariance_DThrust4 = A * _covariance_DThrust4 * A.transpose() + process_noise;
}
bool ThrustKalmanFilter::update_DThrust4(float meas, float measUnc)
{
    _residual_DThrust4 = meas - _thrust_DThrust4(0);
    _innovCov_DThrust4 = _covariance_DThrust4(0, 0) + (measUnc * measUnc);

    matrix::Vector<float, 2> kalmanGain;
    kalmanGain(0) = _covariance_DThrust4(0, 0);
    kalmanGain(1) = _covariance_DThrust4(1, 0);
    kalmanGain /= _innovCov_DThrust4;

    _thrust_DThrust4 += kalmanGain * _residual_DThrust4;
    // _thrust_previous = _thrust;

    // 2*2的单位矩阵
    matrix::Matrix<float, 2, 2> identity;
    identity.identity();

    matrix::Matrix<float, 2, 2> KH;
    KH(0, 0) = kalmanGain(0);
    KH(1, 0) = kalmanGain(1);

    _covariance_DThrust4 = (identity - KH) * _covariance_DThrust4;

    return true;
}
float ThrustKalmanFilter::thrust_kalman_filter_DThrust4(float dt, float inputdata)
{
    dt = (dt < 0.001f) ? 0.01f : dt;

    parameters_update();
	_cov_estimate = _param_td_estimate.get();
    _cov_measure = _param_td_measure.get();

    predict_DThrust4(dt, 0.0f, _cov_estimate);
    update_DThrust4(inputdata, _cov_measure);

    return _thrust_DThrust4(1);
}

// void ThrustKalmanFilter::getState(float &state0, float &state1)
// {
//     state0 = _thrust(0);
//     state1 = _thrust(1);
// }

// void ThrustKalmanFilter::getCovariance(float &cov00, float &cov11)
// {
//     cov00 = _covariance(0, 0);
//     cov11 = _covariance(1, 1);
// }

// void ThrustKalmanFilter::getInnovations(float &innov, float &innovCov)
// {
//     innov = _residual;
//     innovCov = _innovCov;
// }

// 一阶卡尔曼
float ThrustKalmanFilter::BFS1_One_Order_Kalman(float inputdata)
{
	//kalman filiter
	BFS1_forcePredictive_t = BFS1_forcekalman_t_1;
	BFS1_PPredictive_t = BFS1_Pkalman_t_1 + BFS1_Q;
	BFS1_Kg_t = BFS1_PPredictive_t / (BFS1_PPredictive_t + BFS1_R);
	BFS1_forcekalman_t = BFS1_forcePredictive_t + BFS1_Kg_t * (inputdata - BFS1_forcePredictive_t);
	BFS1_PKalman_t = (1 - BFS1_Kg_t) * BFS1_PPredictive_t;
	// 更新当前时刻最优值，作为下一时刻的参考值
	BFS1_forcekalman_t_1 = BFS1_forcekalman_t;
	BFS1_Pkalman_t_1 = BFS1_PKalman_t;

	return BFS1_forcekalman_t;
}

// 输出信号u1进行卡尔曼滤波
float ThrustKalmanFilter::thrust_kalman_filter_u1(float mea, float pre)
{
    parameters_update();
    _cov_estimate_u = _param_estimate_u.get();
    _cov_measure_u = _param_measure_u.get();

    U1_PPredictive_t = U1_Pkalman_t_1 + _cov_estimate_u;
    U1_Kg_t = U1_PPredictive_t / (U1_PPredictive_t + _cov_measure_u);
    U1_forcekalman_t = pre + U1_Kg_t * (mea - pre);
    U1_PKalman_t = (1 - U1_Kg_t) * U1_PPredictive_t;

    U1_Pkalman_t_1 = U1_PKalman_t;

    return U1_forcekalman_t;
}

// 输出信号u2进行卡尔曼滤波
float ThrustKalmanFilter::thrust_kalman_filter_u2(float mea, float pre)
{
    parameters_update();
    _cov_estimate_u = _param_estimate_u.get();
    _cov_measure_u = _param_measure_u.get();

    U2_PPredictive_t = U2_Pkalman_t_1 + _cov_estimate_u;
    U2_Kg_t = U2_PPredictive_t / (U2_PPredictive_t + _cov_measure_u);
    U2_forcekalman_t = pre + U2_Kg_t * (mea - pre);
    U2_PKalman_t = (1 - U2_Kg_t) * U2_PPredictive_t;

    U2_Pkalman_t_1 = U2_PKalman_t;

    return U2_forcekalman_t;
}

// 输出信号u3进行卡尔曼滤波
float ThrustKalmanFilter::thrust_kalman_filter_u3(float mea, float pre)
{
    parameters_update();
    _cov_estimate_u = _param_estimate_u.get();
    _cov_measure_u = _param_measure_u.get();

    U3_PPredictive_t = U3_Pkalman_t_1 + _cov_estimate_u;
    U3_Kg_t = U3_PPredictive_t / (U3_PPredictive_t + _cov_measure_u);
    U3_forcekalman_t = pre + U3_Kg_t * (mea - pre);
    U3_PKalman_t = (1 - U3_Kg_t) * U3_PPredictive_t;

    U3_Pkalman_t_1 = U3_PKalman_t;

    return U3_forcekalman_t;
}

// 输出信号u4进行卡尔曼滤波
float ThrustKalmanFilter::thrust_kalman_filter_u4(float mea, float pre)
{
    parameters_update();
    _cov_estimate_u = _param_estimate_u.get();
    _cov_measure_u = _param_measure_u.get();

    U4_PPredictive_t = U4_Pkalman_t_1 + _cov_estimate_u;
    U4_Kg_t = U4_PPredictive_t / (U4_PPredictive_t + _cov_measure_u);
    U4_forcekalman_t = pre + U4_Kg_t * (mea - pre);
    U4_PKalman_t = (1 - U4_Kg_t) * U4_PPredictive_t;

    U4_Pkalman_t_1 = U4_PKalman_t;

    return U4_forcekalman_t;
}
