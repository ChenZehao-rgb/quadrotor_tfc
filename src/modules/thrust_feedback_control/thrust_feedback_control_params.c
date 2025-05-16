/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * The MAX_VALUE of the Force
 *
 * force_feedback the max_value of the force.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_THRUST_MAX, 2.0f);

/**
 * IOLC Coefficient k0
 *
 * Input-Output Linearization Controller Gain k0.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K0, 1.0f);

/**
 * Kalman Filter Standard deviation of estimation
 *
 * force_feedback Kalman filter standard deviation of estimation. i.e. the standard deviation of the acceleration of the force.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_SD_ES, 5.0f);

/**
 * Kalman Filter Standard deviation of measurement
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_SD_ME, 0.05f);

/**
 * The correction factor of control output
 *
 * Force_d[0~1] = (1-alpha)*omega_d + alpha*omega_d^2, omega is the output of the controller[0~1].
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_ALPHA, 0.0f);

/**
 * IOLC Coefficient kp1
 *
 * Input-Output Linearization Controller Gain kp1.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP1, 5.0f);

/**
 * IOLC Coefficient kp2
 *
 * Input-Output Linearization Controller Gain kp2.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP2, 5.0f);

/**
 * IOLC Coefficient kp3
 *
 * Input-Output Linearization Controller Gain kp3.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP3, 5.0f);

/**
 * IOLC Coefficient kp4
 *
 * Input-Output Linearization Controller Gain kp4.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KP4, 5.0f);