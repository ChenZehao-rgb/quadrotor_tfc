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

px4::AppState ThrustFeedbackControl::appState;

math::LowPassFilter2p<float>	_position_pid_pout_lowpass_filter{100.f, 30.f};
math::LowPassFilter2p<float>	_position_pid_dout_lowpass_filter{100.f, 10.f};

math::LowPassFilter2p<float>	_thrust_desired_1_lowpass_filter{100.f, 5.f};
math::LowPassFilter2p<float>	_thrust_desired_2_lowpass_filter{100.f, 5.f};
math::LowPassFilter2p<float>	_thrust_desired_3_lowpass_filter{100.f, 5.f};
math::LowPassFilter2p<float>	_thrust_desired_4_lowpass_filter{100.f, 5.f};

math::LowPassFilter2p<float>    _output_1_lowpass_filter{100.f, 15.f};
math::LowPassFilter2p<float>    _output_2_lowpass_filter{100.f, 15.f};
math::LowPassFilter2p<float>    _output_3_lowpass_filter{100.f, 15.f};
math::LowPassFilter2p<float>    _output_4_lowpass_filter{100.f, 15.f};

typedef struct
{
    uint64_t last_timestamp;

    float Kp;
    float Ki;
    float Kd;
    float p_out;
    float i_out;
    float limit_i;
    float factorbase_i;
    float d_out;
    float err;
    float last_err;
    float output;

}positional_PID_1;

typedef struct
{
    uint64_t last_timestamp;

    float Kp;
    float Ki;
    float Kd;
    float p_out;
    float i_out;
    float limit_i;
    float factorbase_i;
    float d_out;
    float err;
    float last_err;
    float output;

}positional_PID_2;

typedef struct
{
    uint64_t last_timestamp;

    float Kp;
    float Ki;
    float Kd;
    float p_out;
    float i_out;
    float limit_i;
    float factorbase_i;
    float d_out;
    float err;
    float last_err;
    float output;

}positional_PID_3;

typedef struct
{
    uint64_t last_timestamp;

    float Kp;
    float Ki;
    float Kd;
    float p_out;
    float i_out;
    float limit_i;
    float factorbase_i;
    float d_out;
    float err;
    float last_err;
    float output;

}positional_PID_4;

void Positional_PID_calculate_1(positional_PID_1 *pid)
{
    if (pid == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    float dt = math::constrain(((time_now - pid->last_timestamp) * 1e-6f), 0.001f, 0.05f);

    pid->p_out = pid->Kp * pid->err;
    pid->d_out = pid->Kd * (pid->err - pid->last_err)/dt;

    float i_factor = pid->err / pid->factorbase_i;
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
    pid->i_out = pid->i_out + i_factor * pid->Ki * pid->err * dt;
    pid->i_out = math::constrain(pid->i_out, -pid->limit_i, pid->limit_i);

    pid->p_out = _position_pid_pout_lowpass_filter.apply(pid->p_out);
    pid->d_out = _position_pid_dout_lowpass_filter.apply(pid->d_out);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);

    pid->last_err = pid->err;
    pid->last_timestamp = time_now;
}

void Positional_PID_calculate_2(positional_PID_2 *pid)
{
    if (pid == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    float dt = math::constrain(((time_now - pid->last_timestamp) * 1e-6f), 0.001f, 0.05f);

    pid->p_out = pid->Kp * pid->err;
    pid->d_out = pid->Kd * (pid->err - pid->last_err)/dt;

    float i_factor = pid->err / pid->factorbase_i;
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
    pid->i_out = pid->i_out + i_factor * pid->Ki * pid->err * dt;
    pid->i_out = math::constrain(pid->i_out, -pid->limit_i, pid->limit_i);

    pid->p_out = _position_pid_pout_lowpass_filter.apply(pid->p_out);
    pid->d_out = _position_pid_dout_lowpass_filter.apply(pid->d_out);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);

    pid->last_err = pid->err;
    pid->last_timestamp = time_now;
}

void Positional_PID_calculate_3(positional_PID_3 *pid)
{
    if (pid == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    float dt = math::constrain(((time_now - pid->last_timestamp) * 1e-6f), 0.001f, 0.05f);

    pid->p_out = pid->Kp * pid->err;
    pid->d_out = pid->Kd * (pid->err - pid->last_err)/dt;

    float i_factor = pid->err / pid->factorbase_i;
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
    pid->i_out = pid->i_out + i_factor * pid->Ki * pid->err * dt;
    pid->i_out = math::constrain(pid->i_out, -pid->limit_i, pid->limit_i);

    pid->p_out = _position_pid_pout_lowpass_filter.apply(pid->p_out);
    pid->d_out = _position_pid_dout_lowpass_filter.apply(pid->d_out);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);

    pid->last_err = pid->err;
    pid->last_timestamp = time_now;
}

void Positional_PID_calculate_4(positional_PID_4 *pid)
{
    if (pid == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    float dt = math::constrain(((time_now - pid->last_timestamp) * 1e-6f), 0.001f, 0.05f);

    pid->p_out = pid->Kp * pid->err;
    pid->d_out = pid->Kd * (pid->err - pid->last_err)/dt;

    float i_factor = pid->err / pid->factorbase_i;
    i_factor = math::max(0.0f, 1.f - i_factor * i_factor);
    pid->i_out = pid->i_out + i_factor * pid->Ki * pid->err * dt;
    pid->i_out = math::constrain(pid->i_out, -pid->limit_i, pid->limit_i);

    pid->p_out = _position_pid_pout_lowpass_filter.apply(pid->p_out);
    pid->d_out = _position_pid_dout_lowpass_filter.apply(pid->d_out);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output = (pid->output < 0) ? 0 : ((pid->output > 1.0f) ? 1.0f : pid->output);

    pid->last_err = pid->err;
    pid->last_timestamp = time_now;
}

void ThrustFeedbackControl::parameters_update()
{
    if (_parameter_update_sub.updated()) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
    }
}

int ThrustFeedbackControl::main()
{
    appState.setRunning(true);

    // subscribe to thrust data topic
    int thrustdata_sub_fd = orb_subscribe(ORB_ID(barometric_force_sensor));
    // limit the update rate to 100 Hz
    orb_set_interval(thrustdata_sub_fd, 10);

    // subscribe to thrust desired data topic
    int thrustdesireddata_sub_fd = orb_subscribe(ORB_ID(thrust_desired_data));
    orb_set_interval(thrustdesireddata_sub_fd, 10);

    px4_pollfd_struct_t fds[] = 
    {
        { .fd = thrustdata_sub_fd,   .events = POLLIN },
        { .fd = thrustdesireddata_sub_fd,   .events = POLLIN },
    };

    int error_counter = 0;
    struct barometric_force_sensor_s sensordata;
    struct thrust_desired_data_s thrustdesireddata;
    static positional_PID_1 _pid1 = {};
    static positional_PID_2 _pid2 = {};
    static positional_PID_3 _pid3 = {};
    static positional_PID_4 _pid4 = {};

    _position_pid_pout_lowpass_filter.reset(0.0);
	_position_pid_dout_lowpass_filter.reset(0.0);

    while(appState.isRunning())
    {
        // float time1 = hrt_absolute_time();
        int poll_ret = px4_poll(fds, 2, 1000);

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

                thrustdata.timestamp = hrt_absolute_time();
                static uint64_t last_timestamp = thrustdata.timestamp;
                /*
                    obtain the raw thrust data from the sensor
                */
                thrustdata.thrust_raw_data_1 = sensordata.data1 / 1000.0f;
                thrustdata.thrust_raw_data_2 = sensordata.data2 / 1000.0f;
                thrustdata.thrust_raw_data_3 = sensordata.data3 / 1000.0f;
                thrustdata.thrust_raw_data_4 = sensordata.data4 / 1000.0f;
                thrustdata.thrust_raw_data_1 = (thrustdata.thrust_raw_data_1 > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_data_1 < 0.0f) ? 0.0f : thrustdata.thrust_raw_data_1);
                thrustdata.thrust_raw_data_2 = (thrustdata.thrust_raw_data_2 > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_data_2 < 0.0f) ? 0.0f : thrustdata.thrust_raw_data_2);
                thrustdata.thrust_raw_data_3 = (thrustdata.thrust_raw_data_3 > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_data_3 < 0.0f) ? 0.0f : thrustdata.thrust_raw_data_3);
                thrustdata.thrust_raw_data_4 = (thrustdata.thrust_raw_data_4 > 2.0f) ? 2.0f : ((thrustdata.thrust_raw_data_4 < 0.0f) ? 0.0f : thrustdata.thrust_raw_data_4);

                float dt = (float)(thrustdata.timestamp - last_timestamp) / 1000000.0f;
                // second order kalman filter
                thrustdata.thrust_kalman_filter_data_1 = thrust_kalman_filter.thrust_kalman_filter_BFS1(dt, thrustdata.thrust_raw_data_1);
                thrustdata.thrust_kalman_filter_data_2 = thrust_kalman_filter.thrust_kalman_filter_BFS2(dt, thrustdata.thrust_raw_data_2);
                thrustdata.thrust_kalman_filter_data_3 = thrust_kalman_filter.thrust_kalman_filter_BFS3(dt, thrustdata.thrust_raw_data_3);
                thrustdata.thrust_kalman_filter_data_4 = thrust_kalman_filter.thrust_kalman_filter_BFS4(dt, thrustdata.thrust_raw_data_4);
                thrustdata.thrust_kalman_filter_data_1 = (thrustdata.thrust_kalman_filter_data_1 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_1 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_1);
                thrustdata.thrust_kalman_filter_data_2 = (thrustdata.thrust_kalman_filter_data_2 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_2 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_2);
                thrustdata.thrust_kalman_filter_data_3 = (thrustdata.thrust_kalman_filter_data_3 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_3 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_3);
                thrustdata.thrust_kalman_filter_data_4 = (thrustdata.thrust_kalman_filter_data_4 > 2.0f) ? 2.0f : ((thrustdata.thrust_kalman_filter_data_4 < 0.0f) ? 0.0f : thrustdata.thrust_kalman_filter_data_4);

                last_timestamp = thrustdata.timestamp;
                _thrustdata_pub.publish(thrustdata);
            }

            if (fds[1].revents & POLLIN)
            {
                orb_copy(ORB_ID(thrust_desired_data), thrustdesireddata_sub_fd, &thrustdesireddata);
            }

        }

        parameters_update();

        _thrust_desired(0) = _param_tfc_pwm_to_thrust_factor1.get() * thrustdesireddata.thrust_desired1 - 0.3f;
        _thrust_desired(1) = _param_tfc_pwm_to_thrust_factor2.get() * thrustdesireddata.thrust_desired2 - 0.3f;
        _thrust_desired(2) = _param_tfc_pwm_to_thrust_factor3.get() * thrustdesireddata.thrust_desired3 - 0.3f;
        _thrust_desired(3) = _param_tfc_pwm_to_thrust_factor4.get() * thrustdesireddata.thrust_desired4 - 0.3f;
        _thrust_desired(0) = ( _thrust_desired(0) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(0) < 0.0f) ? 0.0f : _thrust_desired(0));
        _thrust_desired(1) = ( _thrust_desired(1) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(1) < 0.0f) ? 0.0f : _thrust_desired(1));
        _thrust_desired(2) = ( _thrust_desired(2) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(2) < 0.0f) ? 0.0f : _thrust_desired(2));
        _thrust_desired(3) = ( _thrust_desired(3) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(3) < 0.0f) ? 0.0f : _thrust_desired(3));

        // for testing
        // _thrust_desired(0) = _param_tfc_thrust_max.get();
        // _thrust_desired(1) = _param_tfc_thrust_max.get();
        // _thrust_desired(2) = _param_tfc_thrust_max.get();
        // _thrust_desired(3) = _param_tfc_thrust_max.get();

        // thrust feedback control
        _thrust_measure(0) = thrustdata.thrust_kalman_filter_data_2; // motor 1 corresponds to sensor 2
        _thrust_measure(1) = thrustdata.thrust_kalman_filter_data_4; // motor 2 corresponds to sensor 4
        _thrust_measure(2) = thrustdata.thrust_kalman_filter_data_1; // motor 3 corresponds to sensor 1
        _thrust_measure(3) = thrustdata.thrust_kalman_filter_data_3; // motor 4 corresponds to sensor 3

        /*
            thrust closed-loop control
            using Position PID algorithm
        */
        _pid1.Kd = _param_tfc_pid_kp.get();
        _pid1.Ki = _param_tfc_pid_ki.get();
        _pid1.Kd = _param_tfc_pid_kd.get();
        _pid1.limit_i = _param_tfc_pid_lim_i.get();
        _pid1.factorbase_i = _param_tfc_fac_i.get();
        _iolc_u_ff(0) = _param_tfc_iolc_kff_1.get() * thrustdesireddata.thrust_desired1;
        _pid1.err = _thrust_desired(0) - _thrust_measure(0);
        Positional_PID_calculate_1(&_pid1);

        _pid2.Kd = _param_tfc_pid_kp.get();
        _pid2.Ki = _param_tfc_pid_ki.get();
        _pid2.Kd = _param_tfc_pid_kd.get();
        _pid2.limit_i = _param_tfc_pid_lim_i.get();
        _pid2.factorbase_i = _param_tfc_fac_i.get();
        _iolc_u_ff(1) = _param_tfc_iolc_kff_2.get() * thrustdesireddata.thrust_desired2;
        _pid2.err = _thrust_desired(1) - _thrust_measure(1);
        Positional_PID_calculate_2(&_pid2);

        _pid3.Kp = _param_tfc_pid_kp.get();
        _pid3.Ki = _param_tfc_pid_ki.get();
        _pid3.Kd = _param_tfc_pid_kd.get();
        _pid3.limit_i = _param_tfc_pid_lim_i.get();
        _pid3.factorbase_i = _param_tfc_fac_i.get();
        _iolc_u_ff(2) = _param_tfc_iolc_kff_3.get() * thrustdesireddata.thrust_desired3;
        _pid3.err = _thrust_desired(2) - _thrust_measure(2);
        Positional_PID_calculate_3(&_pid3);

        _pid4.Kp = _param_tfc_pid_kp.get();
        _pid4.Ki = _param_tfc_pid_ki.get();
        _pid4.Kd = _param_tfc_pid_kd.get();
        _pid4.limit_i = _param_tfc_pid_lim_i.get();
        _pid4.factorbase_i = _param_tfc_fac_i.get();
        _iolc_u_ff(3) = _param_tfc_iolc_kff_4.get() * thrustdesireddata.thrust_desired4;
        _pid4.err = _thrust_desired(3) - _thrust_measure(3);
        Positional_PID_calculate_4(&_pid4);

        thrustcontroldata.thrust_error1 = _pid1.err;
        thrustcontroldata.thrust_error2 = _pid2.err;
        thrustcontroldata.thrust_error3 = _pid3.err;
        thrustcontroldata.thrust_error4 = _pid4.err;
        thrustcontroldata.thrust_desired1 = _thrust_desired(0);
        thrustcontroldata.thrust_desired2 = _thrust_desired(1);
        thrustcontroldata.thrust_desired3 = _thrust_desired(2);
        thrustcontroldata.thrust_desired4 = _thrust_desired(3);
        thrustcontroldata.thrust_feedback_out1 = _pid1.output;
        thrustcontroldata.thrust_feedback_out2 = _pid2.output;
        thrustcontroldata.thrust_feedback_out3 = _pid3.output;
        thrustcontroldata.thrust_feedback_out4 = _pid4.output;
        _total_output(0) = static_cast<float>(_pid1.output) + _iolc_u_ff(0);
        _total_output(1) = static_cast<float>(_pid2.output) + _iolc_u_ff(1);
        _total_output(2) = static_cast<float>(_pid3.output) + _iolc_u_ff(2);
        _total_output(3) = static_cast<float>(_pid4.output) + _iolc_u_ff(3);

        // _total_output(0) = static_cast<float>(_pid1.output);
        // _total_output(1) = 0;
        // _total_output(2) = 0;
        // _total_output(3) = 0;

        // _total_output(0) = _output_1_lowpass_filter.apply(_total_output(0));
        // _total_output(1) = _output_2_lowpass_filter.apply(_total_output(1));
        // _total_output(2) = _output_3_lowpass_filter.apply(_total_output(2));
        // _total_output(3) = _output_4_lowpass_filter.apply(_total_output(3));

        thrustcontroldata.thrust_feedforward_out1 = _iolc_u_ff(0);
        thrustcontroldata.thrust_feedforward_out2 = _iolc_u_ff(1);
        thrustcontroldata.thrust_feedforward_out3 = _iolc_u_ff(2);
        thrustcontroldata.thrust_feedforward_out4 = _iolc_u_ff(3);
        thrustcontroldata.thrust_control_out1 = _total_output(0);
        thrustcontroldata.thrust_control_out2 = _total_output(1);
        thrustcontroldata.thrust_control_out3 = _total_output(2);
        thrustcontroldata.thrust_control_out4 = _total_output(3);
        thrustcontroldata.thrust_control_out_pwm1 = math::constrain((2.f * _total_output(0) - 1.f), -1.f, 1.f);
        thrustcontroldata.thrust_control_out_pwm2 = math::constrain((2.f * _total_output(1) - 1.f), -1.f, 1.f);
        thrustcontroldata.thrust_control_out_pwm3 = math::constrain((2.f * _total_output(2) - 1.f), -1.f, 1.f);
        thrustcontroldata.thrust_control_out_pwm4 = math::constrain((2.f * _total_output(3) - 1.f), -1.f, 1.f);
        _thrust_kalman_filter_control_out(0) = thrust_kalman_filter.thrust_kalman_filter_u1(thrustcontroldata.thrust_control_out_pwm1, thrustdesireddata.open_loop_control_output1);
        _thrust_kalman_filter_control_out(1) = thrust_kalman_filter.thrust_kalman_filter_u2(thrustcontroldata.thrust_control_out_pwm2, thrustdesireddata.open_loop_control_output2);
        _thrust_kalman_filter_control_out(2) = thrust_kalman_filter.thrust_kalman_filter_u3(thrustcontroldata.thrust_control_out_pwm3, thrustdesireddata.open_loop_control_output3);
        _thrust_kalman_filter_control_out(3) = thrust_kalman_filter.thrust_kalman_filter_u4(thrustcontroldata.thrust_control_out_pwm4, thrustdesireddata.open_loop_control_output4);
        thrustcontroldata.thrust_kalman_filter_control_out_pwm1 = math::constrain(_thrust_kalman_filter_control_out(0), -1.f, 1.f);
        thrustcontroldata.thrust_kalman_filter_control_out_pwm2 = math::constrain(_thrust_kalman_filter_control_out(1), -1.f, 1.f);
        thrustcontroldata.thrust_kalman_filter_control_out_pwm3 = math::constrain(_thrust_kalman_filter_control_out(2), -1.f, 1.f);
        thrustcontroldata.thrust_kalman_filter_control_out_pwm4 = math::constrain(_thrust_kalman_filter_control_out(3), -1.f, 1.f);

        thrustcontroldata.thrust_start = _param_tfc_start.get();
        
        thrustcontroldata.timestamp = hrt_absolute_time();
        _thrustcontroldata_pub.publish(thrustcontroldata);

        px4_usleep(1000);

    }

    return 0;
}
