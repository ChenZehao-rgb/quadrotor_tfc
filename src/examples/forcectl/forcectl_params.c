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
 * @file forcectl_params.c
 * Parameters for force_feedback controller.
 *
 * @author ChangYanchun <changyanchun@sia.cn>
 */

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
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_FOR_MAX, 2.0f);

/**
 * The SOURCE of The Force_Exp
 *
 * force_feedback the source of the force_exp. i.e. 0:from rc[2]; 1: from the actuator_controls0[1]
 *
 * @reboot_required true
 * @boolean
 * @group Force Feedback Control
 */
PARAM_DEFINE_INT32(FORCECTL_FOR_EXP, 0);

/**
 * The proportional of angular acceleration expectation to the force
 *
 * The force(kg) per angular acceleration(rad/s^2).
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_AA2F_K, 0.2f);

/**
 * The weight of the SDOF
 *
 * The number of force value in horizontal hover.
 *
 * @unit kg
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @reboot_required true
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_WEIGHT, 0.7f);

/**
 * Whether to enable weight compensation
 *
 * weight = weight*cos(pitch*enable).
 *
 * @reboot_required true
 * @boolean
 * @group Force Feedback Control
 */
PARAM_DEFINE_INT32(FORCECTL_WEI_COM, 0);

/**
 * The correction factor of control output
 *
 * Force_d[0~1] = (1-alpha)*omega_d + alpha*omega_d^2, omega is the output of the controller[0~1].
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_ALPHA, 0.0f);

/**
 * Battery Compensate Enable
 *
 * 0 : Dot't compensate, 1 : Compensate.
 *
 * @reboot_required true
 * @boolean
 * @group Force Feedback Control
 */
PARAM_DEFINE_INT32(FORCECTL_BAT_COM, 0);

/**
 * PID controller Mode
 *
 * 0 : Incremental PID, 1 : Positional PID.
 *
 * @reboot_required true
 * @boolean
 * @group Force Feedback Control
 */
PARAM_DEFINE_INT32(FORCECTL_PID_MOD, 0);

/**
 * PID controller P gain
 *
 * force_feedback proportional gain.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_PID_P, 0.15f);

/**
 * PID controller I gain
 *
 * force_feedback integral gain.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 4
 * @increment 0.0001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_PID_I, 0.015f);

/**
 * The Limit value of I_out
 *
 * The limit value of the PID controller I_out, i.e. I_out=[-limit,limit].
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_LIM_I, 1.0f);

/**
 * The Base_value of the i_factor
 *
 * Ki = Ki * i_factor, where i_factor = force_error / FORCECTL_FAC_I,
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_FAC_I, 0.7f);

/**
 * PID controller D gain
 *
 * force_feedback differential gain.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_PID_D, 0.003f);

/**
 * ADRC controller b0 gain
 *
 * force_feedback ADRC controller b0 gain.
 *
 * @min 0.0
 * @max 100000
 * @decimal 0
 * @increment 1
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_ADRC_B0, 60000.0f);

/**
 * ADRC controller beta1 gain
 *
 * force_feedback ADRC controller beta1 gain.
 *
 * @min 0.0
 * @max 1000
 * @decimal 1
 * @increment 0.1
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_ADRC_B1, 50.0f);

/**
 * ADRC controller beta2 gain
 *
 * force_feedback ADRC controller beta2 gain.
 *
 * @min 0.0
 * @max 1000
 * @decimal 1
 * @increment 0.1
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_ADRC_B2, 100.0f);

/**
 * ADRC controller beta3 gain
 *
 * force_feedback ADRC controller beta3 gain.
 *
 * @min 0.0
 * @max 1000
 * @decimal 1
 * @increment 0.1
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_ADRC_B3, 80.0f);

/**
 * ADRC controller kp gain
 *
 * force_feedback ADRC controller kp gain.
 *
 * @min 0.0
 * @max 1
 * @decimal 4
 * @increment 0.0001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_ADRC_KP, 0.03f);

/**
 * ADRC controller kd gain
 *
 * force_feedback ADRC controller kd gain.
 *
 * @min 0.0
 * @max 1
 * @decimal 4
 * @increment 0.0001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_ADRC_KD, 0.005f);

/**
 * Kalman Filter Standard deviation of estimation
 *
 * force_feedback Kalman filter standard deviation of estimation. i.e. the standard deviation of the acceleration of the force.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_SD_ES, 5.0f);

/**
 * Kalman Filter Standard deviation of measurement
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_SD_ME, 0.05f);

/**
 * IOLC Coefficient k0
 *
 * Input-Output Linearization Controller Gain k0.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_IOLC_K0, 5.0f);

/**
 * Extended Kalman Filter Observer Standard deviation of estimation
 *
 * force_feedback Kalman filter standard deviation of estimation. i.e. the standard deviation of the acceleration of the force.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 3
 * @increment 0.001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_IOLC_Q, 10.0f);

/**
 * Extended Kalman Filter Observer Standard deviation of estimation
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_IOLC_R, 0.01f);

/**
 * Desired Thrust
 *
 * set desired thrust.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 4
 * @increment 0.0001
 * @group Force Feedback Control
 */
PARAM_DEFINE_FLOAT(FORCECTL_IOLC_D, 0.5f);