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

math::LowPassFilter2p<float>	_position_pid_pout_lowpass_filter{100.f, 30.f};
math::LowPassFilter2p<float>	_position_pid_dout_lowpass_filter{100.f, 10.f};

px4::AppState ThrustFeedbackControl::appState;

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

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff;
	double dt;
    float thrust_desired;
    double motorSpeed_ff;

}IOLC;

typedef struct
{
    uint64_t last_timestamp;

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff;
	double dt;
    float thrust_desired;
    double motorSpeed_ff;

}IOLC2;

typedef struct
{
    uint64_t last_timestamp;

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff;
	double dt;
    float thrust_desired;
    double motorSpeed_ff;

}IOLC3;

typedef struct
{
    uint64_t last_timestamp;

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	float d1, d2;
    float kp, ki;
    float err, err_integral, limit_i;
    double motorSpeed,motorSpeed_dot;
    double u, v, u_ff;
	double dt;
    float thrust_desired;
    double motorSpeed_ff;

}IOLC4;

void IOLC_Calculate(IOLC *iolc)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->err_integral += iolc->err * static_cast<float>(dt);
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);
    iolc->v = iolc->kp * iolc->err + iolc->ki * iolc->err_integral;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = ((iolc->v - f_x) / g_x);
	iolc->u = (iolc->u > 0.3) ? 0.3 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

	/* Using Euler Integration Method to Calculate the Motor Speed */
	double motorSpeed_dot = (iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + motorSpeed_dot*dt;
	iolc->motorSpeed = (iolc->motorSpeed > 600) ? 600 : ((iolc->motorSpeed < 10) ? 10 : iolc->motorSpeed);
}

void IOLC_Calculate2(IOLC2 *iolc)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->err_integral += iolc->err * static_cast<float>(dt);
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);
    iolc->v = iolc->kp * iolc->err + iolc->ki * iolc->err_integral;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = ((iolc->v - f_x) / g_x);
	iolc->u = (iolc->u > 0.3) ? 0.3 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

	/* Using Euler Integration Method to Calculate the Motor Speed */
	double motorSpeed_dot = (iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + motorSpeed_dot*dt;
	iolc->motorSpeed = (iolc->motorSpeed > 600) ? 600 : ((iolc->motorSpeed < 10) ? 10 : iolc->motorSpeed);
}

void IOLC_Calculate3(IOLC3 *iolc)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->err_integral += iolc->err * static_cast<float>(dt);
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);
    iolc->v = iolc->kp * iolc->err + iolc->ki * iolc->err_integral;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = ((iolc->v - f_x) / g_x);
	iolc->u = (iolc->u > 0.3) ? 0.3 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

	/* Using Euler Integration Method to Calculate the Motor Speed */
	double motorSpeed_dot = (iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + motorSpeed_dot*dt;
	iolc->motorSpeed = (iolc->motorSpeed > 600) ? 600 : ((iolc->motorSpeed < 10) ? 10 : iolc->motorSpeed);
}

void IOLC_Calculate4(IOLC4 *iolc)
{
    if (iolc == NULL)
    {
        return;
    }

    uint64_t time_now = hrt_absolute_time();
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.0025f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->err_integral += iolc->err * static_cast<float>(dt);
    iolc->err_integral = math::constrain(iolc->err_integral, -iolc->limit_i, iolc->limit_i);
    iolc->v = iolc->kp * iolc->err + iolc->ki * iolc->err_integral;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = ((iolc->v - f_x) / g_x);
	iolc->u = (iolc->u > 0.3) ? 0.3 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

	/* Using Euler Integration Method to Calculate the Motor Speed */
	double motorSpeed_dot = (iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed + iolc->b0*iolc->u);
	iolc->motorSpeed = iolc->motorSpeed + motorSpeed_dot*dt;
	iolc->motorSpeed = (iolc->motorSpeed > 600) ? 600 : ((iolc->motorSpeed < 10) ? 10 : iolc->motorSpeed);
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
    static IOLC _iolc = {};
    static IOLC2 _iolc2 = {};
    static IOLC3 _iolc3 = {};
    static IOLC4 _iolc4 = {};

    // initialize the IOLC parameters
    _iolc.a1 = iolc_a1;
    _iolc.a2 = iolc_a2;
    _iolc.a3 = iolc_a3;
    _iolc.b0 = iolc_b0;
    _iolc.c1 = iolc_c1;
    _iolc.c2 = iolc_c2;
    _iolc.c3 = iolc_c3;
    _iolc.d1 = iolc_d1;
    _iolc.d2 = iolc_d2;
    _iolc.motorSpeed = motorSpeed_FF;

    _iolc2.a1 = iolc_a1;
    _iolc2.a2 = iolc_a2;
    _iolc2.a3 = iolc_a3;
    _iolc2.b0 = iolc_b0;
    _iolc2.c1 = iolc_c1;
    _iolc2.c2 = iolc_c2;
    _iolc2.c3 = iolc_c3;
    _iolc2.d1 = iolc_d1;
    _iolc2.d2 = iolc_d2;
    _iolc2.motorSpeed = motorSpeed_FF;

    _iolc3.a1 = iolc_a1;
    _iolc3.a2 = iolc_a2;
    _iolc3.a3 = iolc_a3;
    _iolc3.b0 = iolc_b0;
    _iolc3.c1 = iolc_c1;
    _iolc3.c2 = iolc_c2;
    _iolc3.c3 = iolc_c3;
    _iolc3.d1 = iolc_d1;
    _iolc3.d2 = iolc_d2;
    _iolc3.motorSpeed = motorSpeed_FF;

    _iolc4.a1 = iolc_a1;
    _iolc4.a2 = iolc_a2;
    _iolc4.a3 = iolc_a3;
    _iolc4.b0 = iolc_b0;
    _iolc4.c1 = iolc_c1;
    _iolc4.c2 = iolc_c2;
    _iolc4.c3 = iolc_c3;
    _iolc4.d1 = iolc_d1;
    _iolc4.d2 = iolc_d2;
    _iolc4.motorSpeed = motorSpeed_FF;

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

        // obtain the thrust desired data from the flight controller
        // _thrust_desired(0) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired1;
        // _thrust_desired(1) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired2;
        // _thrust_desired(2) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired3;
        // _thrust_desired(3) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired4;
        // _thrust_desired(0) = ( _thrust_desired(0) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(0) < 0.0f) ? 0.0f : _thrust_desired(0));
        // _thrust_desired(1) = ( _thrust_desired(1) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(1) < 0.0f) ? 0.0f : _thrust_desired(1));
        // _thrust_desired(2) = ( _thrust_desired(2) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(2) < 0.0f) ? 0.0f : _thrust_desired(2));
        // _thrust_desired(3) = ( _thrust_desired(3) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(3) < 0.0f) ? 0.0f : _thrust_desired(3));

        _thrust_desired(0) = _param_tfc_pwm_to_thrust_factor1.get() * thrustdesireddata.thrust_desired1 - 0.3f;
        _thrust_desired(1) = _param_tfc_pwm_to_thrust_factor2.get() * thrustdesireddata.thrust_desired2 - 0.3f;
        _thrust_desired(2) = _param_tfc_pwm_to_thrust_factor3.get() * thrustdesireddata.thrust_desired3 - 0.3f;
        _thrust_desired(3) = _param_tfc_pwm_to_thrust_factor4.get() * thrustdesireddata.thrust_desired4 - 0.3f;
        // _thrust_desired(0) = _thrust_desired_1_lowpass_filter.apply(_thrust_desired(0));
        // _thrust_desired(1) = _thrust_desired_2_lowpass_filter.apply(_thrust_desired(1));
        // _thrust_desired(2) = _thrust_desired_3_lowpass_filter.apply(_thrust_desired(2));
        // _thrust_desired(3) = _thrust_desired_4_lowpass_filter.apply(_thrust_desired(3));
        _thrust_desired(0) = ( _thrust_desired(0) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(0) < 0.0f) ? 0.0f : _thrust_desired(0));
        _thrust_desired(1) = ( _thrust_desired(1) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(1) < 0.0f) ? 0.0f : _thrust_desired(1));
        _thrust_desired(2) = ( _thrust_desired(2) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(2) < 0.0f) ? 0.0f : _thrust_desired(2));
        _thrust_desired(3) = ( _thrust_desired(3) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(3) < 0.0f) ? 0.0f : _thrust_desired(3));

        // for testing
        // _thrust_desired(0) = 0.1f;
        // _thrust_desired(1) = 0.3f;
        // _thrust_desired(2) = 0.5f;
        // _thrust_desired(3) = 0.7f;

        // thrust feedback control
        _thrust_measure(0) = thrustdata.thrust_kalman_filter_data_2; // motor 1 corresponds to sensor 2
        _thrust_measure(1) = thrustdata.thrust_kalman_filter_data_4; // motor 2 corresponds to sensor 4
        _thrust_measure(2) = thrustdata.thrust_kalman_filter_data_1; // motor 3 corresponds to sensor 1
        _thrust_measure(3) = thrustdata.thrust_kalman_filter_data_3; // motor 4 corresponds to sensor 3

        /*
            thrust closed-loop control
            using IOLC (Input-Output Linearization Control) algorithm
        */
        // _iolc.kp = _param_tfc_iolc_k.get() * _param_tfc_iolc_kp1.get();
        // _iolc.ki = _param_tfc_iolc_k.get() * _param_tfc_iolc_ki.get();
        _iolc.kp = _param_tfc_iolc_k.get();
        _iolc.ki = _param_tfc_iolc_ki.get();
        _iolc.limit_i = _param_tfc_lim_i.get();
        _iolc.thrust_desired = _thrust_desired(0);
        _iolc_u_ff(0) = _param_tfc_iolc_kff_1.get() * thrustdesireddata.thrust_desired1;
        // _iolc_u_ff(0) = 2.f * thrustdesireddata.thrust_desired1 - 1.f;
        // _iolc_u_ff(0) = (_iolc_u_ff(0) > 1.0f) ? 1.0f : ((_iolc_u_ff(0) < -1.0f) ? -1.0f : _iolc_u_ff(0));
        _iolc.err = _thrust_desired(0) - _thrust_measure(0);
        IOLC_Calculate(&_iolc);

        // _iolc2.kp = _param_tfc_iolc_k.get() * _param_tfc_iolc_kp2.get();
        // _iolc2.ki = _param_tfc_iolc_k.get() * _param_tfc_iolc_ki.get();
        _iolc2.kp = _param_tfc_iolc_k.get();
        _iolc2.ki = _param_tfc_iolc_ki.get();
        _iolc2.limit_i = _param_tfc_lim_i.get();
        _iolc2.thrust_desired = _thrust_desired(1);
        _iolc_u_ff(1) = _param_tfc_iolc_kff_2.get() * thrustdesireddata.thrust_desired2;
        // _iolc_u_ff(1) = 2.f * thrustdesireddata.thrust_desired2 - 1.f;
        // _iolc_u_ff(1) = (_iolc_u_ff(1) > 1.0f) ? 1.0f : ((_iolc_u_ff(1) < -1.0f) ? -1.0f : _iolc_u_ff(1));
        _iolc2.err = _thrust_desired(1) - _thrust_measure(1);
        IOLC_Calculate2(&_iolc2);

        // _iolc3.kp = _param_tfc_iolc_k.get() * _param_tfc_iolc_kp3.get();
        // _iolc3.ki = _param_tfc_iolc_k.get() * _param_tfc_iolc_ki.get();
        _iolc3.kp = _param_tfc_iolc_k.get();
        _iolc3.ki = _param_tfc_iolc_ki.get();
        _iolc3.limit_i = _param_tfc_lim_i.get();
        _iolc3.thrust_desired = _thrust_desired(2);
        _iolc_u_ff(2) = _param_tfc_iolc_kff_3.get() * thrustdesireddata.thrust_desired3;
        // _iolc_u_ff(2) = 2.f * thrustdesireddata.thrust_desired3 - 1.f;
        // _iolc_u_ff(2) = (_iolc_u_ff(2) > 1.0f) ? 1.0f : ((_iolc_u_ff(2) < -1.0f) ? -1.0f : _iolc_u_ff(2));
        _iolc3.err = _thrust_desired(2) - _thrust_measure(2);
        IOLC_Calculate3(&_iolc3);

        // _iolc4.kp = _param_tfc_iolc_k.get() * _param_tfc_iolc_kp4.get();
        // _iolc4.ki = _param_tfc_iolc_k.get() * _param_tfc_iolc_ki.get();
        _iolc4.kp = _param_tfc_iolc_k.get();
        _iolc4.ki = _param_tfc_iolc_ki.get();
        _iolc4.limit_i = _param_tfc_lim_i.get();
        _iolc4.thrust_desired = _thrust_desired(3);
        _iolc_u_ff(3) = _param_tfc_iolc_kff_4.get() * thrustdesireddata.thrust_desired4;
        // _iolc_u_ff(3) = 2.f * thrustdesireddata.thrust_desired4 - 1.f;
        // _iolc_u_ff(3) = (_iolc_u_ff(3) > 1.0f) ? 1.0f : ((_iolc_u_ff(3) < -1.0f) ? -1.0f : _iolc_u_ff(3));
        _iolc4.err = _thrust_desired(3) - _thrust_measure(3);
        IOLC_Calculate4(&_iolc4);

        // thrustcontroldata.motor_speed_dot1 = _iolc.motorSpeed_dot;
        // thrustcontroldata.motor_speed_dot2 = _iolc2.motorSpeed_dot;
        // thrustcontroldata.motor_speed_dot3 = _iolc3.motorSpeed_dot;
        // thrustcontroldata.motor_speed_dot4 = _iolc4.motorSpeed_dot;
        // thrustcontroldata.motor_speed1 = _iolc.motorSpeed;
        // thrustcontroldata.motor_speed2 = _iolc2.motorSpeed;
        // thrustcontroldata.motor_speed3 = _iolc3.motorSpeed;
        // thrustcontroldata.motor_speed4 = _iolc4.motorSpeed;
        thrustcontroldata.deta_t_1 = _iolc.dt;
        thrustcontroldata.deta_t_2 = _iolc2.dt;
        thrustcontroldata.deta_t_3 = _iolc3.dt;
        thrustcontroldata.deta_t_4 = _iolc4.dt;
        thrustcontroldata.thrust_error1 = _iolc.err;
        thrustcontroldata.thrust_error2 = _iolc2.err;
        thrustcontroldata.thrust_error3 = _iolc3.err;
        thrustcontroldata.thrust_error4 = _iolc4.err;
        thrustcontroldata.thrust_desired1 = _thrust_desired(0);
        thrustcontroldata.thrust_desired2 = _thrust_desired(1);
        thrustcontroldata.thrust_desired3 = _thrust_desired(2);
        thrustcontroldata.thrust_desired4 = _thrust_desired(3);
        thrustcontroldata.thrust_feedback_out1 = _iolc.u;
        thrustcontroldata.thrust_feedback_out2 = _iolc2.u;
        thrustcontroldata.thrust_feedback_out3 = _iolc3.u;
        thrustcontroldata.thrust_feedback_out4 = _iolc4.u;
        _total_output(0) = static_cast<float>(_iolc.u) + _iolc_u_ff(0);
        _total_output(1) = static_cast<float>(_iolc2.u) + _iolc_u_ff(1);
        _total_output(2) = static_cast<float>(_iolc3.u) + _iolc_u_ff(2);
        _total_output(3) = static_cast<float>(_iolc4.u) + _iolc_u_ff(3);

        _total_output(0) = _output_1_lowpass_filter.apply(_total_output(0));
        _total_output(1) = _output_2_lowpass_filter.apply(_total_output(1));
        _total_output(2) = _output_3_lowpass_filter.apply(_total_output(2));
        _total_output(3) = _output_4_lowpass_filter.apply(_total_output(3));

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

        // float time2 = hrt_absolute_time();
        //  thrustcontroldata.kp =  (time2 - time1) / 1000.0f;

    }

    return 0;
}
