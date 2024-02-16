/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <lib/drivers/device/Device.hpp>
#include <mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_gps.h>

#include "NMEAParser.hpp"

class TeseoGPS : public ModuleBase<TeseoGPS>, public device::Device
{
public:
	TeseoGPS(const char *path, unsigned baudrate);
	~TeseoGPS() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static TeseoGPS *instantiate(int argc, char *argv[]);

	void run() override;

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	int print_status() override;

private:
	int read_serial_port(uint8_t* buf, size_t size, int timeout_ms);
	int setBaudrate(unsigned baud);

	void update_and_publish();
	void jamming_spoofing_check();

	int             _serial_fd{-1};
	unsigned        _baudrate{0};
	char            _port[20]{};
	bool 			_clock_set{false};

	NMEAParser      _parser{};

	sensor_gps_s 	_gps_report{};
	uint8_t         _spoofing_state{0};
	uint8_t         _jamming_state{0};

	perf_counter_t	_read_error;

	uORB::Publication<sensor_gps_s>    _sensor_gps_pub{ORB_ID(sensor_gps)};
};
