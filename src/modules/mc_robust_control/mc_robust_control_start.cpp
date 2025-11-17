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

static int robust_control_task;

extern "C" __EXPORT int robust_control_main(int argc, char *argv[]);

int robust_control_thread_main(int argc, char *argv[]);

int robust_control_main(int argc, char *argv[])
{
    if (argc < 2)
    {
        PX4_WARN("usage: robust_control {start|stop|status}\n");
        return 1;
    }

    if (!strcmp(argv[1], "start"))
    {
        if (RobustControl::appState.isRunning())
        {
            PX4_INFO("already running\n");
            return 0;
        }

        robust_control_task = px4_task_spawn_cmd("robust_control", // 任务名称
						 SCHED_DEFAULT, // 调度策略
						 SCHED_PRIORITY_MAX - 5, // 任务优先级
						 3000, // 堆栈大小
						 robust_control_thread_main, // 任务入口点函数
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
        
        return 0;
    }

    if (!strcmp(argv[1], "stop")) 
    {
        RobustControl::appState.requestExit();
        RobustControl::appState.setRunning(false);
        return 0;
    }

    if (!strcmp(argv[1], "status"))
    {
        if (RobustControl::appState.isRunning())
        {
            PX4_INFO("running\n");
        }
        else
        {
            PX4_INFO("stopped\n");
        }

        return 0;
    }

    PX4_WARN("unrecognized command\n");
    PX4_WARN("usage: robust_control {start|stop|status}\n");

    return 1;
}

int robust_control_thread_main(int argc, char *argv[])
{
    px4::init(argc, argv, "robust_control");

    printf("Robust Control Start\n");

    RobustControl robustcontrol;
    robustcontrol.main();

    printf("exiting\n");

    return 0;
}