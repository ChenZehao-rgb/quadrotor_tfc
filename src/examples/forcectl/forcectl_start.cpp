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
 * @file forcectl_start_posix.cpp
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */
#include "forcectl_example.hpp"

static int forcectl_task;             /* Handle of forcectl task / thread */

extern "C" __EXPORT int forcectl_main(int argc, char *argv[]);
int forcectl_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: forcectl {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (Forcectl::appState.isRunning()) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		forcectl_task = px4_task_spawn_cmd("forcectl",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 3000,
						 PX4_MAIN,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		Forcectl::appState.requestExit();
		Forcectl::appState.setRunning(false);
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (Forcectl::appState.isRunning()) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	PX4_WARN("usage: forcectl_main {start|stop|status}\n");
	return 1;
}
