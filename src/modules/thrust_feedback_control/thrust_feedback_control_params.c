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
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 1.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_1, 2.312f);

 /**
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 2.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_2, 1.954f);

 /**
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 3.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_3, 2.544f);

 /**
 * PWM to thrust conversion factor
 *
 * PWM to thrust conversion factor of the mortor 4.
 *
 * @unit kg
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PWM_TO_THR_4, 2.517f);

/**
 * IOLC Coefficient K
 *
 * Input-Output Linearization Controller Gain K.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K, 1.0f);

/**
 * IOLC Coefficient ki
 *
 * Input-Output Linearization Controller Gain ki.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KI, 0.1f);

/**
 * The Limit value of I_out
 *
 * The limit value of the PID controller I_out, i.e. I_out=[-limit,limit].
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_LIM_I, 1.0f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_1, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_2, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_3, 0.6f);

/**
 * IOLC Coefficient kff
 *
 * Feedforward Gain kff.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_KFF_4, 0.6f);

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
PARAM_DEFINE_FLOAT(TFC_IOLC_KP1, 0.8f);

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
PARAM_DEFINE_FLOAT(TFC_IOLC_KP2, 0.8f);

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
PARAM_DEFINE_FLOAT(TFC_IOLC_KP3, 0.8f);

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
PARAM_DEFINE_FLOAT(TFC_IOLC_KP4, 0.8f);

/**
 * IOLC Coefficient K1
 *
 * Input-Output Linearization Controller Gain K1.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K1, 1.0f);

/**
 * IOLC Coefficient K2
 *
 * Input-Output Linearization Controller Gain K2.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K2, 1.0f);

/**
 * IOLC Coefficient K3
 *
 * Input-Output Linearization Controller Gain K3.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K3, 1.0f);

/**
 * IOLC Coefficient K4
 *
 * Input-Output Linearization Controller Gain K4.
 *
 * @min 0.0
 * @max 500.0
 * @decimal 1
 * @increment 1
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_IOLC_K4, 1.0f);

/**
 * Mode Switching Coefficient
 *
 * thrust open-loop and thrust closed-loop and thrust output Kalman filter control.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 0.01
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_START, 0.0f);

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
PARAM_DEFINE_FLOAT(TFC_ES_U, 0.1f);

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
PARAM_DEFINE_FLOAT(TFC_ME_U, 0.1f);

/**
 * Motorspeed Desired
 *
 * force_feedback Kalman filter standard deviation of measurement.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_MS_FF, 50.0f);

/**
 * PID Coefficient kp
 *
 * Input-Output Linearization Controller Gain kp.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 3
 * @increment 0.001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PID_KP, 0.05f);

/**
 * PID Coefficient ki
 *
 * Input-Output Linearization Controller Gain ki.
 *
 * @min 0.0
 * @max 5.0_thrust_measure(0) = thrustdata.thrust_kalman_filter_data_1; // motor 1 corresponds to sensor 1
        _thrust_measure(1) = thrustdata.thrust_kalman_filter_data_2; // motor 2 corresponds to sensor 2
        _thrust_measure(2) = thrustdata.thrust_kalman_filter_data_3; // motor 3 corresponds to sensor 3
        _thrust_measure(3) = thrustdata.thrust_kalman_filter_data_4; // motor 4 corresponds to sensor 4
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PID_KI, 0.015f);

/**
 * PID Coefficient kd
 *
 * Input-Output Linearization Controller Gain kd.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.0001
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(TFC_PID_KD, 0.003f);

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
PARAM_DEFINE_FLOAT(TFC_PID_LIM_I, 1.0f);

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
PARAM_DEFINE_FLOAT(TFC_FAC_I, 0.7f);

/**
 * Sensor1 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR1_BIAS1, 260.0f);
/**
 * Sensor1 Bias2
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR1_BIAS2, -635.0f);
/**
 * Sensor2 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR2_BIAS1, 0.0f);
/**
 * Sensor2 Bias2
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR2_BIAS2, -785.0f);

/**
 * Sensor3 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR3_BIAS1, 156.0f);
/**
 * Sensor3 Bias2
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR3_BIAS2, -266.0f);
/**
 * Sensor4 Bias1
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR4_BIAS1, 261.0f);
/**
 * Sensor4 Bias2
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(SENSOR4_BIAS2, -465.0f);

/**
 * Use filtered thrust data or not
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(THR_USE_FIL, 0.0f);
/**
 * The threshold to use feedback control
 *
 * @group Thrust Feedback Control
 */
PARAM_DEFINE_FLOAT(THRE_USE_TFC, 0.15f);
