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

/**
 * @file forcectl_start.cpp
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */

/*这段代码实现了一个名为 forcectl 的应用程序的主函数 forcectl_main，该函数处理应用的启动、停止和状态查询命令。
通过这个主函数，用户可以使用命令行参数来控制这个程序的运行状态。*/

// #include "forcectl_example.hpp"
#include "forcectl_example_iolc_tmech.hpp"


// 这是一个静态的全局变量，用于保存 forcectl 任务或线程的句柄。
// 在PX4中，任务通常是在后台以独立线程的方式运行的，而 forcectl_task 保存了这个任务的句柄，用来跟踪它的状态。
static int forcectl_task;             /* Handle of forcectl task / thread */

// 这是 forcectl 应用的主函数，它处理用户输入的命令行参数，决定如何控制 forcectl 任务的运行。
// 该函数通过 __EXPORT 标记为可导出，使得它可以在PX4的命令行工具中执行。
extern "C" __EXPORT int forcectl_main(int argc, char *argv[]);
int forcectl_main(int argc, char *argv[])
{
	// 这段代码检查是否传入了足够的参数。argc 是命令行参数的数量，
	// 如果小于2（即没有 start、stop 或 status 参数），程序会输出使用说明，并返回错误代码 1。
	if (argc < 2) 
	{
		PX4_WARN("usage: forcectl {start|stop|status}\n");
		return 1;
	}

	// 如果用户输入了 start，该程序会启动 forcectl 任务。
	if (!strcmp(argv[1], "start")) 
	{
		// Forcectl::appState.isRunning() 检查程序是否已经在运行。
		// 如果已经运行，打印“already running”，并返回 0，表示一切正常。
		if (Forcectl::appState.isRunning()) 
		{
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		// 否则，调用 px4_task_spawn_cmd() 创建新的 forcectl 任务，并返回 0。
		// px4_task_spawn_cmd() 是PX4的API，用于创建一个新的任务，
		// 它会启动 forcectl 任务，使用指定的调度策略和优先级运行。
		forcectl_task = px4_task_spawn_cmd("forcectl", // 任务名称
						 SCHED_DEFAULT, // 调度策略
						 SCHED_PRIORITY_MAX - 5, // 任务优先级
						 3000, // 堆栈大小
						 PX4_MAIN, // 任务入口点函数
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr); // 传递给任务的参数

		return 0;
	}

	// 如果用户输入 stop，程序会停止 forcectl 任务。
	if (!strcmp(argv[1], "stop")) 
	{
		// Forcectl::appState.requestExit() 请求退出任务。
		Forcectl::appState.requestExit();
		// Forcectl::appState.setRunning(false) 将任务的状态设置为非运行状态。
		Forcectl::appState.setRunning(false);
		return 0;
	}

	// 处理 status 命令，查询 forcectl 的运行状态。
	if (!strcmp(argv[1], "status")) 
	{
		// 如果任务在运行，打印“is running”。
		if (Forcectl::appState.isRunning()) 
		{
			PX4_INFO("is running\n");
		}
		// 如果任务没有启动，打印“not started”。
		else 
		{
			PX4_INFO("not started\n");
		}

		return 0;
	}

	// 如果传入的命令不是 start、stop 或 status，打印使用说明，并返回错误代码 1。
	PX4_WARN("usage: forcectl_main {start|stop|status}\n");
	return 1;
}
