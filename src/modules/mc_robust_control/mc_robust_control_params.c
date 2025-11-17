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
 * The MAX_VALUE of the Velocity_x
 *
 * robust control the max_value of the velocity_x.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.0001
 * @group Robust Control
 */
PARAM_DEFINE_FLOAT(RC_VEL_MAX_X, 10.0f);

 /**
 * The MAX_VALUE of the Velocity_y
 *
 * robust control the max_value of the velocity_y.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.0001
 * @group Robust Control
 */
PARAM_DEFINE_FLOAT(RC_VEL_MAX_Y, 10.0f);

 /**
 * The MAX_VALUE of the Velocity_z
 *
 * robust control the max_value of the velocity_z.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.0001
 * @group Robust Control
 */
PARAM_DEFINE_FLOAT(RC_VEL_MAX_Z, 4.0f);

 /**
 * The MAX_VALUE of the Acceleration_x
 *
 * robust control the max_value of the Acceleration_x.
 *
 * @min 0.0
 * @max 20.0
 * @decimal 4
 * @increment 0.0001
 * @group Robust Control
 */
PARAM_DEFINE_FLOAT(RC_ACC_MAX_X, 11.0f);

/**
* The MAX_VALUE of the Acceleration_y
*
* robust control the max_value of the Acceleration_y.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_ACC_MAX_Y, 11.0f);

/**
* The MAX_VALUE of the Acceleration_Z
*
* robust control the max_value of the Acceleration_Z.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_ACC_MAX_Z, 11.0f);

/**
* The MAX_VALUE of the Attitude_roll
*
* robust control the max_value of the Attitude_roll.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_ATT_MAX_R, 1.5708f);

/**
* The MAX_VALUE of the Attitude_pitch
*
* robust control the max_value of the Attitude_pitch.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_ATT_MAX_P, 1.5708f);

/**
* The MAX_VALUE of the Attitude_yaw
*
* robust control the max_value of the Attitude_yaw.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_ATT_MAX_Y, 12.5664f);

/**
* The MAX_VALUE of the Rate_roll
*
* robust control the max_value of the Rate_roll.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_RAT_MAX_R, 3.4907f);

/**
* The MAX_VALUE of the Rate_pitch
*
* robust control the max_value of the Rate_pitch.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_RAT_MAX_P, 3.4907f);

/**
* The MAX_VALUE of the Rate_yaw
*
* robust control the max_value of the Rate_yaw.
*
* @min 0.0
* @max 20.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_RAT_MAX_Y, 2.0944f);

/**
* The MAX_VALUE of the Rateacceleration_roll
*
* robust control the max_value of the Rateacceleration_roll.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_RAC_MAX_R, 25.1327f);

/**
* The MAX_VALUE of the Rateacceleration_pitch
*
* robust control the max_value of the Rateacceleration_pitch.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_RAC_MAX_P, 25.1327f);

/**
* The MAX_VALUE of the Rateacceleration_yaw
*
* robust control the max_value of the Rateacceleration_yaw.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_RAC_MAX_Y, 18.8496f);

/**
* The Gravity of the Earth
*
* robust control the Gravity of the Earth.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_GRAV, 9.81f);

/**
* The Mass of the UAV
*
* robust control the Mass of the UAV.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_MASS, 3.78f);

/**
* The KpP Gain of the controller
*
* robust control the KpP Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KP_P, 1.0f);

/**
* The KvP Gain of the controller
*
* robust control the KvP Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KV_P, 1.0f);

/**
* The KaP Gain of the controller
*
* robust control the KaP Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KA_P, 1.0f);

/**
* The KarP Gain of the controller
*
* robust control the KarP Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KAR_P, 1.0f);

/**
* The KpP_x Gain of the controller
*
* robust control the KpP_x Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KP_P_X, 0.95f);

/**
* The KpP_y Gain of the controller
*
* robust control the KpP_y Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KP_P_Y, 0.95f);

/**
* The KpP_z Gain of the controller
*
* robust control the KpP_z Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KP_P_Z, 1.0f);

/**
* The KvP_x Gain of the controller
*
* robust control the KvP_x Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KV_P_X, 1.5f);

/**
* The KvP_y Gain of the controller
*
* robust control the KvP_y Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KV_P_Y, 1.5f);

/**
* The KvP_z Gain of the controller
*
* robust control the KvP_z Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KV_P_Z, 4.0f);

/**
* The KaP_roll Gain of the controller
*
* robust control the KaP_roll Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KA_P_R, 5.2f);

/**
* The KaP_pitch Gain of the controller
*
* robust control the KaP_pitch Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KA_P_P, 5.2f);

/**
* The KaP_yaw Gain of the controller
*
* robust control the KaP_yaw Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KA_P_Y, 6.5f);

/**
* The KarP_roll Gain of the controller
*
* robust control the KarP_roll Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KAR_P_R, 0.08f);

/**
* The KarP_pitch Gain of the controller
*
* robust control the KarP_pitch Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KAR_P_P, 0.08f);

/**
* The KarP_yaw Gain of the controller
*
* robust control the KarP_yaw Gain of the controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_KAR_P_Y, 0.08f);

/**
* The K Gain of the feedforward controller
*
* robust control the K Gain of the feedforward controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_K_FF, 0.8f);

/**
* The K Gain of the feedback controller
*
* robust control the K Gain of the feedback controller.
*
* @min 0.0
* @max 50.0
* @decimal 4
* @increment 0.0001
* @group Robust Control
*/
PARAM_DEFINE_FLOAT(RC_K_FB, 0.2f);

/**
 * Mode Switching Coefficient
 *
 * robust control or PID control.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 0.01
 * @group Robust Control
 */
PARAM_DEFINE_FLOAT(RC_START, 0.0f);
