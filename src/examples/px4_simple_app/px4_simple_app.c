/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <assert.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	printf("Initializing /dev/ttyS2...\n");

	int uartfd = open("/dev/ttyS2", O_RDWR | O_NOCTTY); // TELEM2 port
	assert(uartfd >= 0);
	// manage terminal settings
	int termios_state_ttyS2;
	struct termios existing_config_ttyS2;
	// get existing terminal config and store it.
	assert((termios_state_ttyS2 = tcgetattr(uartfd, &existing_config_ttyS2)) >= 0);
	struct termios config_ttyS2;
	// duplicate into the new config
	tcgetattr(uartfd, &config_ttyS2);
	// memcpy(config_ttyS2, existing_config_ttyS2);

	// clear ONLCR flag
	config_ttyS2.c_oflag &= ~ONLCR;
	// set baud rate
	assert(cfsetispeed(&config_ttyS2, B921600) >= 0 || cfsetospeed(&config_ttyS2, B921600) >= 0);
	// go ahead and set the config i am setting up
	assert((termios_state_ttyS2 = tcsetattr(uartfd, TCSANOW, &config_ttyS2)) >= 0);
	printf("Got to 76\n");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(sensor_sub_fd, 2);
	printf("Got to 81\n");

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		{ .fd = other_sub_fd,   .events = POLLIN },
		*/
	};

	int error_counter = 0;
	int byteswritten = 0;

	for (int i = 0; i < 50000; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 1000);
		if (!(i%1000)) {
			printf("Written %d bytes, i=%d\n", byteswritten, i);
		}

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[px4_simple_app] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[px4_simple_app] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);


				char* buf = "abcdef\n";
				// int printlen = sprintf(buf, "[px4_simple_app] -- on uart -- 
				//* Accelerometer: "
				// 	"\t%8.4f\t%8.4f\t%8.4f\n",
				// 	(double)raw.accelerometer_m_s2[0],
				// 	(double)raw.accelerometer_m_s2[1],
				// 	(double)raw.accelerometer_m_s2[2]);
				// write(uartfd, buf, 7);
				byteswritten += dprintf(uartfd, "Accelerometer: \t%8.4f\t%8.4f\t%8.4f\n",
						(double)raw.accelerometer_m_s2[0],
						(double)raw.accelerometer_m_s2[1],
						(double)raw.accelerometer_m_s2[2]);
			}
			/* there could be more file descriptors here, in the form like:
			if (fds[1..n].revents & POLLIN) {}
			*/
		}
	}

	// cleanup serial port
	tcsetattr(uartfd, TCSANOW, &existing_config_ttyS2);
	close(uartfd);

	return 0;
}
