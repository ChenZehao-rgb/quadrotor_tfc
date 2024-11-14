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

#include "thrust_feedback_control.hpp"

math::LowPassFilter2p<float>	_positional_pid_pout_lowpass_filter{100.f, 30.f};
math::LowPassFilter2p<float>	_positional_pid_dout_lowpass_filter{100.f, 10.f};

px4::AppState ThrustFeedbackControl::appState;

// typedef struct
// {
//     uint64_t pre_timestamp;

//     float kp;
//     float ki;
//     float kd;

//     float factorbase_i;
//     float limit_i;

//     // float dt;
//     float pre_error;
//     float error;
//     float integral;
//     float derivative;

//     float thrust_exp;
//     float thrust_mea;

//     float p_out;
//     float i_out;
//     float d_out;
//     float output;
// }positional_PID;

// void Positional_PID_Calculate(positional_PID *pos_pid)
// {
//     if (pos_pid == NULL)
//     {
//         return;
//     }

//     pos_pid->error = pos_pid->thrust_exp - pos_pid->thrust_mea;
//     pos_pid->integral += pos_pid->error * pos_pid->dt;
//     pos_pid->derivative = (pos_pid->error - pos_pid->pre_error) / pos_pid->dt;

//     pos_pid->p_out = pos_pid->kp * pos_pid->error;
//     pos_pid->i_out = pos_pid->ki * pos_pid->integral;
//     pos_pid->d_out = pos_pid->kd * pos_pid->derivative;

//     pos_pid->output = pos_pid->p_out + pos_pid->i_out + pos_pid->d_out;

//     pos_pid->pre_error = pos_pid->error;
// }

typedef struct
{
    uint64_t pre_timestamp;

    float kp;
    float ki;
    float kd;

    float factorbase_i;
    float limit_i;

    // float dt;
    matrix::Vector<float, 4> pre_error;
    matrix::Vector<float, 4> error;
    matrix::Vector<float, 4> integral;
    matrix::Vector<float, 4> derivative;

    matrix::Vector<float, 4> thrust_exp;
    matrix::Vector<float, 4> thrust_mea;

    matrix::Vector<float, 4> p_out;
    matrix::Vector<float, 4> i_out;
    matrix::Vector<float, 4> d_out;
    matrix::Vector<float, 4> output;
}positional_PID;

void Positional_PID_Calculate(positional_PID *pos_pid, int _rotor_count)
{
    if (pos_pid == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    float dt = math::constrain(((time_now - pos_pid->pre_timestamp) * 1e-6f), 0.001f, 0.05f);

    pos_pid->error(_rotor_count) = pos_pid->thrust_exp(_rotor_count) - pos_pid->thrust_mea(_rotor_count);
    pos_pid->derivative(_rotor_count) = (pos_pid->error(_rotor_count) - pos_pid->pre_error(_rotor_count)) / dt;
    float i_factor = pos_pid->error(_rotor_count) / pos_pid->factorbase_i;
    i_factor = math::max(0.0f, 1.0f - i_factor * i_factor);
    pos_pid->integral(_rotor_count) += i_factor * pos_pid->error(_rotor_count) * dt;

    pos_pid->p_out(_rotor_count) = pos_pid->kp * pos_pid->error(_rotor_count);
    pos_pid->i_out(_rotor_count) = pos_pid->ki * pos_pid->integral(_rotor_count);
    pos_pid->d_out(_rotor_count) = pos_pid->kd * pos_pid->derivative(_rotor_count);

    pos_pid->p_out(_rotor_count) = _positional_pid_pout_lowpass_filter.apply(pos_pid->p_out(_rotor_count));
    pos_pid->i_out(_rotor_count) = math::constrain(pos_pid->i_out(_rotor_count), -pos_pid->limit_i, pos_pid->limit_i);
    pos_pid->d_out(_rotor_count) = _positional_pid_dout_lowpass_filter.apply(pos_pid->d_out(_rotor_count));

    pos_pid->output(_rotor_count) = pos_pid->p_out(_rotor_count) + pos_pid->i_out(_rotor_count) + pos_pid->d_out(_rotor_count);
    pos_pid->output(_rotor_count) = (pos_pid->output(_rotor_count) < 0) ? 0 : ((pos_pid->output(_rotor_count) > 1.0f) ? 1.0f : pos_pid->output(_rotor_count));

    pos_pid->pre_error(_rotor_count) = pos_pid->error(_rotor_count);
    pos_pid->pre_timestamp = time_now;
}

int ThrustFeedbackControl::main()
{
    appState.setRunning(true);

    // 订阅气压式力传感器数据
    int thrustdata_sub_fd = orb_subscribe(ORB_ID(barometric_force_sensor));
    // 更新频率50Hz
    orb_set_interval(thrustdata_sub_fd, 20);

    // 订阅各轴期望升力
    int thrustdesireddata_sub_fd = orb_subscribe(ORB_ID(thrust_desired_data));
    orb_set_interval(thrustdesireddata_sub_fd, 20);

    // 订阅总的期望升力
    int forceexp_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
    orb_set_interval(forceexp_sub_fd, 20);

    px4_pollfd_struct_t fds[] = 
    {
        { .fd = thrustdata_sub_fd,   .events = POLLIN },
        { .fd = thrustdesireddata_sub_fd,   .events = POLLIN },
        { .fd = forceexp_sub_fd,   .events = POLLIN },
    };

    int error_counter = 0;
    int _rotor_count;
    // 气压式力传感器的读数
    struct barometric_force_sensor_s sensordata;
    struct thrust_desired_data_s thrustdesireddata;
    static positional_PID pos_pid = {};
    struct actuator_controls_s force_exp = {};

    while(appState.isRunning())
    {
        int poll_ret = px4_poll(fds, 3, 1000);

        if (poll_ret == 0)
        {
            PX4_ERR("Got no data within a second");
        }
        else if (poll_ret < 0)
        {
            if (error_counter < 10 || error_counter % 50 ==0)
            {
                PX4_ERR("ERROR RETURN VALUE FROM POLL(): %d", poll_ret);
            }

            error_counter++;
        }
        else 
        {
            if (fds[0].revents & POLLIN)
            {
                orb_copy(ORB_ID(barometric_force_sensor), thrustdata_sub_fd, &sensordata);
            }

            if (fds[1].revents & POLLIN)
            {
                orb_copy(ORB_ID(thrust_desired_data), thrustdesireddata_sub_fd, &thrustdesireddata);
            }

            if (fds[2].revents & POLLIN)
            {
                orb_copy(ORB_ID(actuator_controls_0), forceexp_sub_fd, &force_exp);
            }
            // PX4_INFO("Desired_Force : \t%.3f", static_cast<double>(force_exp.control[3]));
        }

        /*
            读取四个气压式力传感器的数据
        */
        thrustdata.thrust_raw_data_1 = sensordata.data1 / 1000.0f;
        thrustdata.thrust_raw_data_2 = sensordata.data2 / 1000.0f;
        thrustdata.thrust_raw_data_3 = sensordata.data3 / 1000.0f;
        thrustdata.thrust_raw_data_4 = sensordata.data4 / 1000.0f;

        /*
            进行卡尔曼滤波处理
        */
        // 求解时间间隔
        hrt_abstime now = hrt_absolute_time();
        thrustdata.timestamp = now;
        static uint64_t last_timestamp = now;
        // dt 单位为秒
        float dt = (float)(thrustdata.timestamp - last_timestamp) / 1000000.0f;
        // 二阶卡尔曼滤波
        thrustdata.thrust_kalman_filter_data_1 = thrust_kalman_filter.thrust_kalman_filter_BFS1(dt, thrustdata.thrust_raw_data_1);
        thrustdata.thrust_kalman_filter_data_2 = thrust_kalman_filter.thrust_kalman_filter_BFS2(dt, thrustdata.thrust_raw_data_2);
        thrustdata.thrust_kalman_filter_data_3 = thrust_kalman_filter.thrust_kalman_filter_BFS3(dt, thrustdata.thrust_raw_data_3);
        thrustdata.thrust_kalman_filter_data_4 = thrust_kalman_filter.thrust_kalman_filter_BFS4(dt, thrustdata.thrust_raw_data_4);
        last_timestamp = thrustdata.timestamp;
        _thrustdata_pub.publish(thrustdata);
        // PX4_INFO("Raw_Data & Kalman_Data :\t%.3fkg\t%.3fkg\t%.3fkg\t%.3fkg\t%.3fkg\t%.3fkg\t%.3fkg\t%.3fkg", 
        //         static_cast<double>(thrustdata.thrust_raw_data_1), 
        //         static_cast<double>(thrustdata.thrust_kalman_filter_data_1), 
        //         static_cast<double>(thrustdata.thrust_raw_data_2), 
        //         static_cast<double>(thrustdata.thrust_kalman_filter_data_2),
        //         static_cast<double>(thrustdata.thrust_raw_data_3), 
        //         static_cast<double>(thrustdata.thrust_kalman_filter_data_3),
        //         static_cast<double>(thrustdata.thrust_raw_data_4), 
        //         static_cast<double>(thrustdata.thrust_kalman_filter_data_4));
        PX4_INFO("Desired_Thrust : \t%.3f\t%.3f\t%.3f\t%.3f", 
                static_cast<double>(thrustdesireddata.thrust_desired1),
                static_cast<double>(thrustdesireddata.thrust_desired2),
                static_cast<double>(thrustdesireddata.thrust_desired3),
                static_cast<double>(thrustdesireddata.thrust_desired4));
        // px4_usleep(10000);

        /*
            力闭环控制 PID控制
        */
        _thrust_desired(0) = thrustdesireddata.thrust_desired1;
        _thrust_desired(1) = thrustdesireddata.thrust_desired2;
        _thrust_desired(2) = thrustdesireddata.thrust_desired3;
        _thrust_desired(3) = thrustdesireddata.thrust_desired4;

        _thrust_measure(0) = thrustdata.thrust_kalman_filter_data_1;
        _thrust_measure(1) = thrustdata.thrust_kalman_filter_data_2;
        _thrust_measure(2) = thrustdata.thrust_kalman_filter_data_3;
        _thrust_measure(3) = thrustdata.thrust_kalman_filter_data_4;

        thrustcontroldata.kp = pos_PID_kp;
        thrustcontroldata.ki = pos_PID_ki;
        thrustcontroldata.kd = pos_PID_kd;

        pos_pid.kp = thrustcontroldata.kp;
        pos_pid.ki = thrustcontroldata.ki;
        pos_pid.kd = thrustcontroldata.kd;

        for (_rotor_count = 0; _rotor_count < 4; _rotor_count ++)
        {
            pos_pid.thrust_exp(_rotor_count) = Thrust_Max * _thrust_desired(_rotor_count);
            pos_pid.thrust_mea(_rotor_count) = _thrust_measure(_rotor_count);
            Positional_PID_Calculate(&pos_pid, _rotor_count);
        }
        
        // pos_pid.dt = dt;
        thrustcontroldata.thrust_error1 = pos_pid.error(0);
        thrustcontroldata.thrust_error2 = pos_pid.error(1);
        thrustcontroldata.thrust_error3 = pos_pid.error(2);
        thrustcontroldata.thrust_error4 = pos_pid.error(3);

        thrustcontroldata.thrust_control_out1 = pos_pid.output(0);
        thrustcontroldata.thrust_control_out2 = pos_pid.output(1);
        thrustcontroldata.thrust_control_out3 = pos_pid.output(2);
        thrustcontroldata.thrust_control_out4 = pos_pid.output(3);

        _thrustcontroldata_pub.publish(thrustcontroldata);
    }

    return 0;
}