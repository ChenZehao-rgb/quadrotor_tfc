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

#include "mc_robust_control.hpp"

#undef _K

using matrix::Quatf;
using matrix::Eulerf;

px4::AppState RobustControl::appState;

void RobustControl::parameters_update()
{
    if (_parameter_update_sub.updated()) 
    {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
    }
}

void computeWandDotW(float roll_des, float pitch_des,
                     float dot_roll_des, float dot_pitch_des,
                     matrix::Matrix3f &W, matrix::Matrix3f &dot_W)
{
    // =========================
    // 计算 W
    // =========================
    W(0,0) = 1.0f;
    W(0,1) = tanf(pitch_des) * sinf(roll_des);
    W(0,2) = tanf(pitch_des) * cosf(roll_des);

    W(1,0) = 0.0f;
    W(1,1) = cosf(roll_des);
    W(1,2) = -sinf(roll_des);

    W(2,0) = 0.0f;
    W(2,1) = sinf(roll_des) / cosf(pitch_des);
    W(2,2) = cosf(roll_des) / cosf(pitch_des);

    // =========================
    // 计算 dot_W
    // =========================
    float dot_W_12 = (dot_pitch_des * sinf(roll_des)) / (cosf(pitch_des) * cosf(pitch_des))
                   + dot_roll_des * tanf(pitch_des) * cosf(roll_des);

    float dot_W_13 = (dot_pitch_des * cosf(roll_des)) / (cosf(pitch_des) * cosf(pitch_des))
                   - dot_roll_des * tanf(pitch_des) * sinf(roll_des);

    float dot_W_22 = -dot_roll_des * sinf(roll_des);
    float dot_W_23 = -dot_roll_des * cosf(roll_des);

    float dot_W_32 = (dot_roll_des * cosf(roll_des)) / cosf(pitch_des)
                   + (dot_pitch_des * sinf(roll_des) * sinf(pitch_des)) / (cosf(pitch_des) * cosf(pitch_des));

    float dot_W_33 = -(dot_roll_des * sinf(roll_des)) / cosf(pitch_des)
                   + (dot_pitch_des * cosf(roll_des) * sinf(pitch_des)) / (cosf(pitch_des) * cosf(pitch_des));

    dot_W(0,0) = 0.0f;   dot_W(0,1) = dot_W_12; dot_W(0,2) = dot_W_13;
    dot_W(1,0) = 0.0f;   dot_W(1,1) = dot_W_22; dot_W(1,2) = dot_W_23;
    dot_W(2,0) = 0.0f;   dot_W(2,1) = dot_W_32; dot_W(2,2) = dot_W_33;
}

// 输入：I, W, dot_W, omega, angularacc_sp
// 输出：M (3x1 torque vector)
matrix::Vector3f computeM(const matrix::Matrix3f &I,
                          const matrix::Matrix3f &W,
                          const matrix::Matrix3f &dot_W,
                          const matrix::Vector3f &omega,
                          const matrix::Vector3f &angularacc_sp)
{
    // I*omega
    matrix::Vector3f I_omega = I * omega;

    // cross(omega, I*omega)
    matrix::Vector3f gyro_term = omega.cross(I_omega);

    // W * I^{-1} * gyro_term
    matrix::Matrix3f I_inv = I.I();   // 求逆
    matrix::Vector3f term1 = W * (I_inv * gyro_term);

    // -dot_W * omega
    matrix::Vector3f term2 = -(dot_W * omega);

    // 合并
    matrix::Vector3f inner = term2 + term1 + angularacc_sp;

    // M = I * W^{-1} * inner
    matrix::Matrix3f W_inv = W.I();   // W 的逆
    matrix::Vector3f M = I * (W_inv * inner);

    return M;
}

int RobustControl::main()
{
    appState.setRunning(true);

    // 定义惯性矩阵 I
    matrix::Matrix3f I;  
    I.setZero();  // 先全部置 0
    I(0,0) = 0.07739f;
    I(1,1) = 0.07739f;
    I(2,2) = 0.1375f;

    while(appState.isRunning())
    {
        // 更新参数
        parameters_update();

        // 更新位置误差话题
        _robust_control_position_error_sub.update(&_position_error);
        // 更新速度误差话题
        _robust_control_velocity_error_sub.update(&_velocity_error);
        // 更新姿态误差话题
        _robust_control_attitude_error_sub.update(&_attitude_error);
        // 更新角速度误差话题
        _robust_control_rate_error_sub.update(&_rate_error);
        // 更新期望加速度话题
        _vehicle_local_position_setpoint_sub.update(&_vehicle_local_position_setpoint);
        // 更新期望姿态话题
        _vehicle_attitude_setpoint_sub.update(&_vehicle_attitude_setpoint);
        // 更新当前姿态话题
        _vehicle_attitude_sub.update(&_vehicle_attitude);
        // 更新前馈力和力矩话题
        _robust_control_data_pid_sub.update(&_feedforward_force_moment);

        /*将误差数据存入矩阵*/
        // 位置误差
        _pos_error(0) = _position_error.position_error_x;
        _pos_error(1) = _position_error.position_error_y;
        _pos_error(2) = _position_error.position_error_z;
        // 速度误差
        _vel_error(0) = _velocity_error.velocity_error_x;
        _vel_error(1) = _velocity_error.velocity_error_y;
        _vel_error(2) = _velocity_error.velocity_error_z;
        // 姿态误差
        _att_error(0) = _attitude_error.attitude_error_roll;
        _att_error(1) = _attitude_error.attitude_error_pitch;
        _att_error(2) = _attitude_error.attitude_error_yaw;
        // 角速度误差
        _rat_error(0) = _rate_error.rate_error_roll;
        _rat_error(1) = _rate_error.rate_error_pitch;
        _rat_error(2) = _rate_error.rate_error_yaw;
        // 期望加速度
        _acc_sp(0) = _vehicle_local_position_setpoint.acceleration[0];
        _acc_sp(1) = _vehicle_local_position_setpoint.acceleration[1];
        _acc_sp(2) = _vehicle_local_position_setpoint.acceleration[2];
        // 期望角速度
        _rat_sp(0) = _rate_error.rate_sp_roll;
        _rat_sp(1) = _rate_error.rate_sp_pitch;
        _rat_sp(2) = _rate_error.rate_sp_yaw;
        // 当前角速度
        _rat(0) = _rate_error.rate_roll;
        _rat(1) = _rate_error.rate_pitch;
        _rat(2) = _rate_error.rate_yaw;
        // 期望姿态
        Quatf q_d(_vehicle_attitude_setpoint.q_d);
        Eulerf euler_d(q_d);
        _att_sp(0) = euler_d.phi();
        _att_sp(1) = euler_d.theta();
        _att_sp(2) = euler_d.psi();
        // 当前姿态
        Quatf q(_vehicle_attitude.q);
        Eulerf euler(q);
        _att(0) = euler.phi();
        _att(1) = euler.theta();
        _att(2) = euler.psi();
        // 前馈力和力矩
        _feedforward_force_moment_vector(0) = _feedforward_force_moment.pid_desired_total_force;
        _feedforward_force_moment_vector(1) = _feedforward_force_moment.pid_desired_torque_x;
        _feedforward_force_moment_vector(2) = _feedforward_force_moment.pid_desired_torque_y;
        _feedforward_force_moment_vector(3) = _feedforward_force_moment.pid_desired_torque_z;

        /* 参数列表读取 */
        // 加速度最大值
        _acceleration_max(0) = _param_rc_acc_x_max.get();
        _acceleration_max(1) = _param_rc_acc_y_max.get();
        _acceleration_max(2) = _param_rc_acc_z_max.get();
        // 重力加速度
        float grav = _param_rc_grav.get();
        // 质量
        float mass = _param_rc_mass.get();
        // 增益
        _KpP = _param_rc_kp_p.get();
        _KvP = _param_rc_kv_p.get();
        _KaP = _param_rc_ka_p.get();
        _KarP = _param_rc_kar_p.get();
        _Kff = _param_rc_k_ff.get();
        _Kfb = _param_rc_k_fb.get();
        _KpP_xyz(0) = _param_rc_kp_p_x.get();
        _KpP_xyz(1) = _param_rc_kp_p_y.get();
        _KpP_xyz(2) = _param_rc_kp_p_z.get();
        _KvP_xyz(0) = _param_rc_kv_p_x.get();
        _KvP_xyz(1) = _param_rc_kv_p_y.get();
        _KvP_xyz(2) = _param_rc_kv_p_z.get();
        _KaP_rpy(0) = _param_rc_ka_p_r.get();
        _KaP_rpy(1) = _param_rc_ka_p_p.get();
        _KaP_rpy(2) = _param_rc_ka_p_y.get();
        _KarP_rpy(0) = _param_rc_kar_p_r.get();
        _KarP_rpy(1) = _param_rc_kar_p_p.get();
        _KarP_rpy(2) = _param_rc_kar_p_y.get();
        /* 更新参数增益 */
        float KpP_Coefficients  = _KpP;
        float KvP_Coefficients  = _KvP;
        float KaP_Coefficients  = _KaP;
        float KarP_Coefficients = _KarP;
        // 定义对角矩阵
        matrix::SquareMatrix<float, 3> KpP = KpP_Coefficients * matrix::diag(matrix::Vector3f(_KpP_xyz(0), _KpP_xyz(1), _KpP_xyz(2)));
        matrix::SquareMatrix<float, 3> KvP = KvP_Coefficients * matrix::diag(matrix::Vector3f(_KvP_xyz(0), _KvP_xyz(1), _KvP_xyz(2)));
        matrix::SquareMatrix<float, 3> KaP = KaP_Coefficients * matrix::diag(matrix::Vector3f(_KaP_rpy(0), _KaP_rpy(1), _KaP_rpy(2)));
        matrix::SquareMatrix<float, 3> KarP = KarP_Coefficients * matrix::diag(matrix::Vector3f(_KarP_rpy(0), _KarP_rpy(1), _KarP_rpy(2)));
        matrix::SquareMatrix<float, 3> K0; // 默认初始化为零矩阵
        // 构造 K 矩阵 (6x12)
        _K.setZero();
        // 手动赋值 KpP 到 (0:2,0:2)
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                _K(i, j) = KpP(i, j);     // 左上角
            }
        }
        // KvP 到 (0:2,3:5)
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                _K(i, j + 3) = KvP(i, j);
            }
        }
        // KaP 到 (3:5,6:8)
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                _K(i + 3, j + 6) = KaP(i, j);
            }
        }
        // KarP 到 (3:5,9:11)
        for (int i = 0; i < 3; i++) 
        {
            for (int j = 0; j < 3; j++) 
            {
                _K(i + 3, j + 9) = KarP(i, j);
            }
        }
        // 提取子矩阵
        _K_pos = _K.slice<3,6>(0,0);  // 第1~3行, 第1~6列
        _K_att = _K.slice<3,6>(3,6);  // 第4~6行, 第7~12列

        /*******************************************************/
        /*******************************************************/
        /******************** 计算期望总升力 ********************/
        /*******************************************************/
        /*******************************************************/
        // 构造误差向量 (6x1)
        _pos_err(0) = _pos_error(0);
        _pos_err(1) = _pos_error(1);
        _pos_err(2) = _pos_error(2);
        _pos_err(3) = _vel_error(0);
        _pos_err(4) = _vel_error(1);
        _pos_err(5) = _vel_error(2);
        /****** 计算虚拟控制量v1(3x1)，即加速度期望 ******/
        v1 = _K_pos * _pos_err;
        matrix::Vector3f _acc_des = _acc_sp; // 期望加速度
        ControlMath::addIfNotNanVector3f(_acc_des, v1);
        _acc_des(0) = math::constrain(_acc_des(0), 0.0f, _acceleration_max(0));
        _acc_des(1) = math::constrain(_acc_des(1), 0.0f, _acceleration_max(1));
        _acc_des(2) = math::constrain(_acc_des(2), -_acceleration_max(2), _acceleration_max(2));
        // 计算四个旋翼所需的总升力
        matrix::Vector3f F_w = (matrix::Vector3f(0.0f, 0.0f, -grav) + _acc_des) * mass;
        // 总升力的模长
        _robust_control_data.desired_total_force = F_w.norm();
        _robust_control_data.desired_total_force = math::constrain(_robust_control_data.desired_total_force, 0.0f, 1.0f);

        /*****************************************************/
        /*****************************************************/
        /******************** 计算期望力矩 ********************/
        /*****************************************************/
        /*****************************************************/
        // 构造误差向量 (6x1)
        _att_err(0) = _att_error(0);
        _att_err(1) = _att_error(1);
        _att_err(2) = _att_error(2);
        _att_err(3) = _rat_error(0);
        _att_err(4) = _rat_error(1);
        _att_err(5) = _rat_error(2);
        /****** 计算虚拟控制量v2(3x1)，即角加速度期望 ******/
        v2 = _K_att * _att_err;
        // 期望角加速度限幅
        matrix::Vector3f _ang_acc_des = v2;
        _ang_acc_des(0) = math::constrain(_ang_acc_des(0), -_acceleration_max(0), _acceleration_max(0));
        _ang_acc_des(1) = math::constrain(_ang_acc_des(1), -_acceleration_max(1), _acceleration_max(1));
        _ang_acc_des(2) = math::constrain(_ang_acc_des(2), -_acceleration_max(2), _acceleration_max(2));
        /****** 计算姿态运动学矩阵 ******/
        computeWandDotW(_att_sp(0), _att_sp(1), _rat_sp(0), _rat_sp(1), W, dot_W);
        matrix::Vector3f M = computeM(I, W, dot_W, _rat, _ang_acc_des);
        _robust_control_data.desired_torque_x = M(0);
        _robust_control_data.desired_torque_y = M(1);
        _robust_control_data.desired_torque_z = M(2);
        _robust_control_data.desired_torque_x = math::constrain(_robust_control_data.desired_torque_x, -1.0f, 1.0f);
        _robust_control_data.desired_torque_y = math::constrain(_robust_control_data.desired_torque_y, -1.0f, 1.0f);
        _robust_control_data.desired_torque_z = math::constrain(_robust_control_data.desired_torque_z, -1.0f, 1.0f);

        _robust_control_data.total_desired_total_force = _feedforward_force_moment_vector(0);
        _robust_control_data.total_desired_torque_x = _Kff * _feedforward_force_moment_vector(1) + _Kfb * _robust_control_data.desired_torque_x;
        _robust_control_data.total_desired_torque_y = _Kff * _feedforward_force_moment_vector(2) + _Kfb * _robust_control_data.desired_torque_y;
        _robust_control_data.total_desired_torque_z = _Kff * _feedforward_force_moment_vector(3) + _Kfb * _robust_control_data.desired_torque_z;

        // 鲁棒控制器开关
        _robust_control_data.robust_control_start = _param_rc_start.get();
        // 发布数据
        _robust_control_data.timestamp = hrt_absolute_time();
        _robust_control_data_pub.publish(_robust_control_data);

        px4_usleep(1000);

    }

    return 0;
}
