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

typedef struct
{
    uint64_t last_timestamp;

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	double d0, d1, d2, d3;
    float k0;
    float err;
    double motorSpeed;
    double u, v;
	double dt;

}IOLC;

typedef struct
{
    uint64_t last_timestamp;

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	double d0, d1, d2, d3;
    float k0;
    float err;
    double motorSpeed;
    double u, v;
	double dt;

}IOLC2;

typedef struct
{
    uint64_t last_timestamp;

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	double d0, d1, d2, d3;
    float k0;
    float err;
    double motorSpeed;
    double u, v;
	double dt;

}IOLC3;

typedef struct
{
    uint64_t last_timestamp;

    // IOLC
	double a1, a2, a3;
    double b0;
	double c1, c2, c3;
	double d0, d1, d2, d3;
    float k0;
    float err;
    double motorSpeed;
    double u, v;
	double dt;

}IOLC4;

void IOLC_Calculate(IOLC *iolc)
{
    if (iolc == NULL)
    {
        return;
    }

    // 获取当前时间戳，单位为微秒
    uint64_t time_now = hrt_absolute_time();
    // 限制dt在0.001秒至0.05秒之间，防止时间间隔过短或过长导致不稳定
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.05f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->v = iolc->k0 * iolc->err;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = (iolc->v - f_x) / g_x;
	iolc->u = (iolc->u > 0.8) ? 0.8 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

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

    // 获取当前时间戳，单位为微秒
    uint64_t time_now = hrt_absolute_time();
    // 限制dt在0.001秒至0.05秒之间，防止时间间隔过短或过长导致不稳定
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.05f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->v = iolc->k0 * iolc->err;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = (iolc->v - f_x) / g_x;
	iolc->u = (iolc->u > 0.8) ? 0.8 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

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

    // 获取当前时间戳，单位为微秒
    uint64_t time_now = hrt_absolute_time();
    // 限制dt在0.001秒至0.05秒之间，防止时间间隔过短或过长导致不稳定
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.05f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->v = iolc->k0 * iolc->err;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = (iolc->v - f_x) / g_x;
	iolc->u = (iolc->u > 0.8) ? 0.8 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

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

    // 获取当前时间戳，单位为微秒
    uint64_t time_now = hrt_absolute_time();
    // 限制dt在0.001秒至0.05秒之间，防止时间间隔过短或过长导致不稳定
    double dt = math::constrain(((time_now - iolc->last_timestamp) * 1e-6f), 0.001f, 0.05f);
    iolc->last_timestamp = time_now;
    iolc->dt = dt;

    // virtual control law
    iolc->v = iolc->k0 * iolc->err;
    // control law
	double f_x = (3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1)
		*(iolc->a2*iolc->motorSpeed*iolc->motorSpeed + iolc->a1*iolc->motorSpeed);
	double g_x = (iolc->b0*(3*iolc->c3*iolc->motorSpeed*iolc->motorSpeed + 2*iolc->c2*iolc->motorSpeed + iolc->c1));
    iolc->u = (iolc->v - f_x) / g_x;
	iolc->u = (iolc->u > 0.8) ? 0.8 : ((iolc->u < 0.0) ? 0.0 : iolc->u);

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

    // 订阅气压式力传感器数据
    int thrustdata_sub_fd = orb_subscribe(ORB_ID(barometric_force_sensor));
    // 更新频率50Hz
    orb_set_interval(thrustdata_sub_fd, 20);

    // 订阅各轴期望升力
    int thrustdesireddata_sub_fd = orb_subscribe(ORB_ID(thrust_desired_data));
    orb_set_interval(thrustdesireddata_sub_fd, 20);

    // 订阅总的期望升力
    // int forceexp_sub_fd = orb_subscribe(ORB_ID(actuator_controls_0));
    // orb_set_interval(forceexp_sub_fd, 20);

    /* subscribe to actuator_controls_3 topic */
	 int forceexp_sub_rc_fd = orb_subscribe(ORB_ID(actuator_controls_3));
	 /* limit the update rate to 50 Hz */
	 orb_set_interval(forceexp_sub_rc_fd, 20);

    /* subscribe to rc_channels topic */
	 int rc_sub_fd = orb_subscribe(ORB_ID(rc_channels));
	 /* limit the update rate to 50 Hz */
	 orb_set_interval(rc_sub_fd, 20);

    px4_pollfd_struct_t fds[] = 
    {
        { .fd = thrustdata_sub_fd,   .events = POLLIN },
        { .fd = thrustdesireddata_sub_fd,   .events = POLLIN },
        // { .fd = forceexp_sub_fd,   .events = POLLIN },
        { .fd = forceexp_sub_rc_fd,   .events = POLLIN },
    };

    int error_counter = 0;
    // int _rotor_count;
    // 气压式力传感器的读数
    struct barometric_force_sensor_s sensordata;
    struct thrust_desired_data_s thrustdesireddata;
    static IOLC _iolc = {};
    static IOLC2 _iolc2 = {};
    static IOLC3 _iolc3 = {};
    static IOLC4 _iolc4 = {};
    // struct actuator_controls_s force_exp{};

    // 初始化参数
    _iolc.a1 = iolc_a1;
    _iolc.a2 = iolc_a2;
    _iolc.a3 = iolc_a3;
    _iolc.b0 = iolc_b0;
    _iolc.c1 = iolc_c1;
    _iolc.c2 = iolc_c2;
    _iolc.c3 = iolc_c3;

    _iolc2.a1 = iolc_a1;
    _iolc2.a2 = iolc_a2;
    _iolc2.a3 = iolc_a3;
    _iolc2.b0 = iolc_b0;
    _iolc2.c1 = iolc_c1;
    _iolc2.c2 = iolc_c2;
    _iolc2.c3 = iolc_c3;

    _iolc3.a1 = iolc_a1;
    _iolc3.a2 = iolc_a2;
    _iolc3.a3 = iolc_a3;
    _iolc3.b0 = iolc_b0;
    _iolc3.c1 = iolc_c1;
    _iolc3.c2 = iolc_c2;
    _iolc3.c3 = iolc_c3;

    _iolc4.a1 = iolc_a1;
    _iolc4.a2 = iolc_a2;
    _iolc4.a3 = iolc_a3;
    _iolc4.b0 = iolc_b0;
    _iolc4.c1 = iolc_c1;
    _iolc4.c2 = iolc_c2;
    _iolc4.c3 = iolc_c3;

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
                // 求解时间间隔
                thrustdata.timestamp = hrt_absolute_time();
                static uint64_t last_timestamp = thrustdata.timestamp;
                /*
                    读取四个气压式力传感器的数据
                */
                thrustdata.thrust_raw_data_1 = sensordata.data1 / 1000.0f;
                thrustdata.thrust_raw_data_2 = sensordata.data2 / 1000.0f;
                thrustdata.thrust_raw_data_3 = sensordata.data3 / 1000.0f;
                thrustdata.thrust_raw_data_4 = sensordata.data4 / 1000.0f;
                // dt 单位为秒
                float dt = (float)(thrustdata.timestamp - last_timestamp) / 1000000.0f;
                // 二阶卡尔曼滤波
                thrustdata.thrust_kalman_filter_data_1 = thrust_kalman_filter.thrust_kalman_filter_BFS1(dt, thrustdata.thrust_raw_data_1);
                thrustdata.thrust_kalman_filter_data_2 = thrust_kalman_filter.thrust_kalman_filter_BFS2(dt, thrustdata.thrust_raw_data_2);
                thrustdata.thrust_kalman_filter_data_3 = thrust_kalman_filter.thrust_kalman_filter_BFS3(dt, thrustdata.thrust_raw_data_3);
                thrustdata.thrust_kalman_filter_data_4 = thrust_kalman_filter.thrust_kalman_filter_BFS4(dt, thrustdata.thrust_raw_data_4);
                last_timestamp = thrustdata.timestamp;
                _thrustdata_pub.publish(thrustdata);
            }

            if (fds[1].revents & POLLIN)
            {
                orb_copy(ORB_ID(thrust_desired_data), thrustdesireddata_sub_fd, &thrustdesireddata);
            }

            if (fds[2].revents & POLLIN)
            {
                // orb_copy(ORB_ID(actuator_controls_0), forceexp_sub_fd, &force_exp);
                orb_copy(ORB_ID(actuator_controls_3), forceexp_sub_rc_fd, &force_exp_from_rc);
            }
            // PX4_INFO("Desired_Force : \t%.3f", static_cast<double>(force_exp.control[3]));
        }

        parameters_update();
        /*
            获取期望升力和滤波后的升力测量值
        */
        // _thrust_desired(0) = _param_tfc_thrust_max.get() * force_exp_from_rc.control[3];
        // _thrust_desired(1) = _param_tfc_thrust_max.get() * force_exp_from_rc.control[3];
        // _thrust_desired(2) = _param_tfc_thrust_max.get() * force_exp_from_rc.control[3];
        // _thrust_desired(3) = _param_tfc_thrust_max.get() * force_exp_from_rc.control[3];
        // _thrust_desired(0) = _param_tfc_thrust_max.get();
        // _thrust_desired(1) = _param_tfc_thrust_max.get();
        // _thrust_desired(2) = _param_tfc_thrust_max.get();
        // _thrust_desired(3) = _param_tfc_thrust_max.get();
        _thrust_desired(0) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired1;
        _thrust_desired(1) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired2;
        _thrust_desired(2) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired3;
        _thrust_desired(3) = _param_tfc_thrust_max.get() * thrustdesireddata.thrust_desired4;
        _thrust_desired(0) = ( _thrust_desired(0) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(0) < 0.0f) ? 0.0f : _thrust_desired(0));
        _thrust_desired(1) = ( _thrust_desired(1) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(1) < 0.0f) ? 0.0f : _thrust_desired(1));
        _thrust_desired(2) = ( _thrust_desired(2) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(2) < 0.0f) ? 0.0f : _thrust_desired(2));
        _thrust_desired(3) = ( _thrust_desired(3) > _param_tfc_thrust_max.get()) ? _param_tfc_thrust_max.get() : ((_thrust_desired(3) < 0.0f) ? 0.0f : _thrust_desired(3));

        // 无力反馈
        // _thrust_measure(0) = thrustdata.thrust_kalman_filter_data_2; // 1号电机对应2号传感器
        // _thrust_measure(1) = thrustdata.thrust_kalman_filter_data_4; // 2号电机对应4号传感器
        // _thrust_measure(2) = thrustdata.thrust_kalman_filter_data_1; // 3号电机对应1号传感器
        // _thrust_measure(3) = thrustdata.thrust_kalman_filter_data_3; // 4号电机对应3号传感器

        // 力反馈
        _thrust_measure(0) = thrustdata.thrust_kalman_filter_data_4; // 1号电机对应4号传感器
        _thrust_measure(1) = thrustdata.thrust_kalman_filter_data_2; // 2号电机对应2号传感器
        _thrust_measure(2) = thrustdata.thrust_kalman_filter_data_1; // 3号电机对应1号传感器
        _thrust_measure(3) = thrustdata.thrust_kalman_filter_data_3; // 4号电机对应3号传感器

        orb_copy(ORB_ID(rc_channels), rc_sub_fd, &rc_channals_data);
        if((rc_channals_data.channels[5] > -0.3f)&&(rc_channals_data.channels[5] < 0.3f))
        {
            // float forcectl_alpha = _param_tfc_alpha.get();
            // forcectl_alpha = (forcectl_alpha < 0.01f) ? 0.01f : ((forcectl_alpha > 1.0f) ? 1.0f : forcectl_alpha);
            // for (_rotor_count = 0; _rotor_count < 4; _rotor_count ++)
            // {
            //     _control_output(_rotor_count) = ((float)sqrt((1.0f - forcectl_alpha)*(1.0f - forcectl_alpha) + 
            //         4.0f*forcectl_alpha*_thrust_desired(_rotor_count)) + (forcectl_alpha - 1.0f))/(2.0f * forcectl_alpha);
            // }
            // thrustcontroldata.thrust_control_out1 = _control_output(0);
            // thrustcontroldata.thrust_control_out2 = _control_output(1);
            // thrustcontroldata.thrust_control_out3 = _control_output(2);
            // thrustcontroldata.thrust_control_out4 = _control_output(3);
        }
		else if(rc_channals_data.channels[5] > 0.3f)
        // if(rc_channals_data.channels[5] > 0.3f)
        {
            /*
                力闭环控制 IOLC控制
            */
            _iolc.k0 = _param_tfc_iolc_k0.get();
            _iolc.err = _thrust_desired(0) - _thrust_measure(0);
            IOLC_Calculate(&_iolc);

            _iolc2.k0 = _param_tfc_iolc_k0.get();
            _iolc2.err = _thrust_desired(1) - _thrust_measure(1);
            IOLC_Calculate2(&_iolc2);

            _iolc3.k0 = _param_tfc_iolc_k0.get();
            _iolc3.err = _thrust_desired(2) - _thrust_measure(2);
            IOLC_Calculate3(&_iolc3);

            _iolc4.k0 = _param_tfc_iolc_k0.get();
            _iolc4.err = _thrust_desired(3) - _thrust_measure(3);
            IOLC_Calculate4(&_iolc4);

            thrustcontroldata.thrust_error1 = _iolc.err;
            thrustcontroldata.thrust_error2 = _iolc2.err;
            thrustcontroldata.thrust_error3 = _iolc3.err;
            thrustcontroldata.thrust_error4 = _iolc4.err;
            thrustcontroldata.thrust_desired1 = _thrust_desired(0);
            thrustcontroldata.thrust_desired2 = _thrust_desired(1);
            thrustcontroldata.thrust_desired3 = _thrust_desired(2);
            thrustcontroldata.thrust_desired4 = _thrust_desired(3);
            thrustcontroldata.thrust_control_out1 = _iolc.u;
            thrustcontroldata.thrust_control_out2 = _iolc2.u;
            thrustcontroldata.thrust_control_out3 = _iolc3.u;
            thrustcontroldata.thrust_control_out4 = _iolc4.u;

            // _iolc.err = _thrust_desired(1) - _thrust_measure(1);
            // IOLC_Calculate(&_iolc);
            // thrustcontroldata.thrust_error2 = _iolc.err;
            // thrustcontroldata.thrust_desired2 = _thrust_desired(1);
            // thrustcontroldata.thrust_control_out2 = _iolc.u;
            // thrustcontroldata.thrust_control_out1 = 0.0f;
            // thrustcontroldata.thrust_control_out3 = 0.0f;
            // thrustcontroldata.thrust_control_out4 = 0.0f;

            // _iolc.err = _thrust_desired(2) - _thrust_measure(2);
            // IOLC_Calculate(&_iolc);
            // thrustcontroldata.thrust_error3 = _iolc.err;
            // thrustcontroldata.thrust_desired3 = _thrust_desired(2);
            // thrustcontroldata.thrust_control_out3 = _iolc.u;
            // thrustcontroldata.thrust_control_out1 = 0.0f;
            // thrustcontroldata.thrust_control_out2 = 0.0f;
            // thrustcontroldata.thrust_control_out4 = 0.0f;

            // _iolc.err = _thrust_desired(3) - _thrust_measure(3);
            // IOLC_Calculate(&_iolc);
            // thrustcontroldata.thrust_error4 = _iolc.err;
            // thrustcontroldata.thrust_desired4 = _thrust_desired(3);
            // thrustcontroldata.thrust_control_out4 = _iolc.u;
            // thrustcontroldata.thrust_control_out1 = 0.0f;
            // thrustcontroldata.thrust_control_out2 = 0.0f;
            // thrustcontroldata.thrust_control_out3 = 0.0f;

        }
        else if(rc_channals_data.channels[5] < -0.3f)
        {
            thrustcontroldata.thrust_control_out1 = 0.0f;
            thrustcontroldata.thrust_control_out2 = 0.0f;
            thrustcontroldata.thrust_control_out3 = 0.0f;
            thrustcontroldata.thrust_control_out4 = 0.0f;
        }
        
        thrustcontroldata.timestamp = hrt_absolute_time();
        _thrustcontroldata_pub.publish(thrustcontroldata);

        px4_usleep(1000);
    }

    return 0;
}
