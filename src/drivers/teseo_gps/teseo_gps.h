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

// TODO: audit includes
#include <termios.h>
#include <cstring>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gnss_relative.h>

#include "NMEAParser.hpp"

class TeseoGPS : public ModuleBase<TeseoGPS>, public device::Device
{
public:
	enum class Instance : uint8_t {
		Main = 0,
		Secondary,
		Count
	};

	TeseoGPS(const char *path, Instance instance, unsigned baudrate);
	~TeseoGPS() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int task_spawn(int argc, char *argv[], Instance instance);

	/** @see ModuleBase */
	static TeseoGPS *instantiate(int argc, char *argv[]);
	static TeseoGPS *instantiate(int argc, char *argv[], Instance instance);

	void run() override;

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	static int run_trampoline_secondary(int argc, char *argv[]);

	int print_status() override;

private:
	int						_serial_fd{-1};
	unsigned				_baudrate{0};
	char					_port[20] {};					///< device / serial port path

	NMEAParser 			   _parser{};


	bool					_healthy{false};				///< flag to signal if the GPS is ok


	sensor_gps_s			_report_gps_pos{};				///< uORB topic for gps position
	uint8_t                 _spoofing_state{0};                             ///< spoofing state
	uint8_t                 _jamming_state{0};                              ///< jamming state

	uORB::PublicationMulti<sensor_gps_s>	_report_gps_pos_pub{ORB_ID(sensor_gps)};	///< uORB pub for gps position
	uORB::PublicationMulti<sensor_gnss_relative_s> _sensor_gnss_relative_pub{ORB_ID(sensor_gnss_relative)};

	float				_rate{0.0f};					///< position update rate
	unsigned			_num_bytes_read{0}; 				///< counter for number of read bytes from the UART (within update interval)
	unsigned			_rate_reading{0}; 				///< reading rate in B/s

	const Instance 			_instance;

	static px4::atomic_bool _is_gps_main_advertised; ///< for the second gps we want to make sure that it gets instance 1
	/// and thus we wait until the first one publishes at least one message.

	static px4::atomic<TeseoGPS*> _secondary_instance;


	void 				publish();
	void 				publishRelativePosition(sensor_gnss_relative_s &gnss_relative);

	/**
	 * This is an abstraction for the poll on serial used.
	 *
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout in ms
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int read_serial_port(uint8_t* buf, size_t size, int timeout);

	int setBaudrate(unsigned baud);

	static constexpr int SET_CLOCK_DRIFT_TIME_S{5};			///< RTC drift time when time synchronization is needed (in seconds)
};
