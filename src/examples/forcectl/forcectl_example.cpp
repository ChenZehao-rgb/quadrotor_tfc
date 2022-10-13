/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file forcectl_start.c
 * Text application example for force feedback control
 *
 * @author Yanchun Chang <changyanchun@sia.cn>
 */

#include "forcectl_example.h"

px4::AppState Forcectl::appState;

int Forcectl::main()
{
	appState.setRunning(true);

	/* subscribe to forcectl_forcedata topic */
	int forcedata_sub_fd = orb_subscribe(ORB_ID(forcectl_forcedata));
	/* limit the update rate to 50 Hz */
	orb_set_interval(forcedata_sub_fd, 20);

	/* subscribe to actuator_controls_3 topic */
	int forceexp_sub_fd = orb_subscribe(ORB_ID(actuator_controls_3));
	/* limit the update rate to 50 Hz */
	orb_set_interval(forceexp_sub_fd, 20);

	/* subscribe to rc_channels topic */
	int pid_sub_fd = orb_subscribe(ORB_ID(rc_channels));
	/* limit the update rate to 1 Hz */
	orb_set_interval(pid_sub_fd, 20);

	/* advertise forcectl_controldata topic */
	struct forcectl_controldata_s control_data;
	memset(&control_data, 0, sizeof(control_data));
	orb_advert_t controldata_pub = orb_advertise(ORB_ID(forcectl_controldata), &control_data);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = forcedata_sub_fd,   .events = POLLIN },
		{ .fd = forceexp_sub_fd,   .events = POLLIN },
	};

	struct forcectl_forcedata_s forcedata = {};
	struct actuator_controls_s force_exp = {};
	struct rc_channels_s rc_channals_data = {};
	float force_max = 1.0f;

	while(appState.isRunning()){
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 2, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret > 0) {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(forcectl_forcedata), forcedata_sub_fd, &forcedata);
				// PX4_INFO("The force_raw data: %8.4f, %8.4f\t",
				// 	 (double)forcedata.force_raw_data,
				// 	 (double)forcedata.force_filtered_data);
			}

			if (fds[1].revents & POLLIN) {
				/* obtained data for the first file descriptor */

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(actuator_controls_3), forceexp_sub_fd, &force_exp);
				control_data.force_exp_data = force_max*force_exp.control[3];
				//PX4_INFO("The force_exp data: %8.4f\t", (double)forcectl_force_exp);
			}

		}

		orb_copy(ORB_ID(rc_channels), pid_sub_fd, &rc_channals_data);
		control_data.force_controls_p = (rc_channals_data.channels[6] + 1.0f)/2.0f;
		control_data.force_controls_i = (rc_channals_data.channels[7] + 1.0f)/2.0f;
		control_data.force_controls_d = (rc_channals_data.channels[8] + 1.0f)/2.0f;

		float force_error = control_data.force_exp_data - forcedata.force_filtered_data;
		control_data.force_controls_data = control_data.force_controls_p*force_error/force_max;

		control_data.timestamp = hrt_absolute_time();
		orb_publish(ORB_ID(forcectl_controldata), controldata_pub, &control_data);

		px4_usleep(10000);
	}

	return 0;
}
