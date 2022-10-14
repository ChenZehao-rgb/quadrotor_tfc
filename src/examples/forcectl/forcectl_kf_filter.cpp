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
 * @file forcectl_kf_filter.cpp
 * The code of KF_filter for force feedback control
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */

#include "forcectl_kf_filter.hpp"

ForcectlKfFilter::ForcectlKfFilter(float covInit)
{
	_force(0) = 0.0f;
	_force(1) = 0.0f;
	_covariance(0,0) = covInit;
	_covariance(1,1) = covInit;
}

void ForcectlKfFilter::predict(float dt, float acc, float acc_unc)
{
	_force(0) += _force(1) * dt + dt * dt / 2 * acc;
	_force(1) += acc * dt;

	matrix::Matrix<float, 2, 2> A; // propagation matrix
	A(0, 0) = 1;
	A(1, 1) = 1;
	A(0, 1) = dt;

	matrix::Matrix<float, 2, 1> G; // noise model
	G(0, 0) = dt * dt / 2;
	G(1, 0) = dt;

	matrix::Matrix<float, 2, 2> process_noise = G * G.transpose() * (acc_unc * acc_unc);

	_covariance = A * _covariance * A.transpose() + process_noise;
}

bool ForcectlKfFilter::update(float meas, float measUnc)
{
	// H = [1, 0]
	_residual = meas - _force(0);

	// H * P * H^T simply selects P(0,0)
	_innovCov = _covariance(0, 0) + (measUnc * measUnc);

	// outlier rejection
	float beta = _residual / _innovCov * _residual;

	// 5% false alarm probability
	if (beta > 3.84f) {
		return false;
	}

	matrix::Vector<float, 2> kalmanGain;
	kalmanGain(0) = _covariance(0, 0);
	kalmanGain(1) = _covariance(1, 0);
	kalmanGain /= _innovCov;

	_force += kalmanGain * _residual;

	matrix::Matrix<float, 2, 2> identity;
	identity.identity();

	matrix::Matrix<float, 2, 2> KH; // kalmanGain * H
	KH(0, 0) = kalmanGain(0);
	KH(1, 0) = kalmanGain(1);

	_covariance = (identity - KH) * _covariance;

	return true;
}

void ForcectlKfFilter::getState(float &state0, float &state1)
{
	state0 = _force(0);
	state1 = _force(1);
}

void ForcectlKfFilter::getCovariance(float &cov00, float &cov11)
{
	cov00 = _covariance(0, 0);
	cov11 = _covariance(1, 1);
}

void ForcectlKfFilter::getInnovations(float &innov, float &innovCov)
{
	innov = _residual;
	innovCov = _innovCov;
}

float ForcectlKfFilter::force_kf_filter(float dt, float inputdata)
{
	dt = (dt < 0.001f) ? 0.01f : dt;
	predict(dt, 0.0f, _cov_estimate);
	update(inputdata, _cov_measure);

	return _force(0);
}
