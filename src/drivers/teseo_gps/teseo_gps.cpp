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

#include "teseo_gps.h"

TeseoGPS::TeseoGPS(const char* device_path, unsigned baudrate) :
	Device(MODULE_NAME),
	_baudrate(baudrate),
	_read_error(perf_alloc(PC_COUNT, MODULE_NAME": read error"))
{
	// store port name
	strncpy(_port, device_path, sizeof(_port) - 1);
	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	_gps_report.heading = NAN;
	_gps_report.heading_offset = NAN;

	set_device_bus_type(device::Device::DeviceBusType::DeviceBusType_SERIAL);

	char c = _port[strlen(_port) - 1]; // last digit of path (eg /dev/ttyS2)
	set_device_bus(c - 48); // sub 48 to convert char to integer
}

TeseoGPS::~TeseoGPS()
{
	perf_free(_read_error);
}

void TeseoGPS::run()
{
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (_serial_fd < 0) {
		PX4_ERR("failed to open %s err: %d", _port, errno);
		return;
	}

	setBaudrate(_baudrate);

	set_device_type(DRV_GPS_DEVTYPE_NMEA);

	// TODO: configure() : send commands to configure the modules (constellations, rate, filter, etc)

	uint8_t rxbuf[1024] {};
	int timeout_ms = 100;

	while (!should_exit()) {

		int bytes_read = read_serial_port(rxbuf, sizeof(rxbuf), timeout_ms);

		if (bytes_read > 0) {
			int parsed_count = _parser.parse(rxbuf, bytes_read);

			if (parsed_count > 0) {
				update_and_publish();
			}
		}
	}
}

void TeseoGPS::update_and_publish()
{
	RMC_Data* rmc = _parser.RMC();
	GGA_Data* gga = _parser.GGA();
	// VTG_Data* vtg = _parser.VTG();
	GST_Data* gst = _parser.GST();
	GSA_Data* gsa = _parser.GSA();

	// TODO: do we want to use this?
	// (void)vtg;

	double lat = gga->lat;
	double lon = gga->lon;

	///// GGA
	if (gga->ns == 'S') {
		lat = -lat;
	}

	if (gga->ew == 'W') {
		lon = -lon;
	}

	// Convert to PX4 definitions
	switch (gga->fix_quality) {
	case 1:
		// 3D fix
		_gps_report.fix_type = 3;
		break;

	case 2:
		// Differential
		_gps_report.fix_type = 4;
		break;

	case 6:
		// Dead reckoning we'll call 2D fix
		_gps_report.fix_type = 2;
		break;
	}

	_gps_report.longitude_deg = int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0;
	_gps_report.latitude_deg = int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0;
	_gps_report.hdop = gga->hdop;
	_gps_report.altitude_msl_m = (double)gga->alt;
	_gps_report.altitude_ellipsoid_m = (double)(gga->alt + gga->geo_sep);

	// Only report sats if there's a fix
	if (_gps_report.fix_type > 1) {
		_gps_report.satellites_used = gga->sats;

	} else {
		_gps_report.satellites_used = 0;
	}

	///// GST
	_gps_report.eph = sqrtf(gst->lat_err * gst->lat_err + gst->lon_err * gst->lon_err);
	_gps_report.epv = gst->alt_err;
	_gps_report.ehpe = gst->ehpe;

	///// GSA
	_gps_report.hdop = gsa->hdop;
	_gps_report.vdop = gsa->vdop;

	///// RMC
	float velocity_ms = rmc->speed / 1.9438445f; // knots to m/s
	float track_rad = rmc->track_good * M_PI_F / 180.0f; // rad in range [0, 2pi]

	_gps_report.vel_m_s = velocity_ms;
	_gps_report.vel_n_m_s = velocity_ms * cosf(track_rad);
	_gps_report.vel_e_m_s = velocity_ms * sinf(track_rad);
	_gps_report.cog_rad = track_rad;

	_gps_report.vel_ned_valid = rmc->mode != 'N';

	// If RMC says No Fix
	if (rmc->status == 'V') {
		_gps_report.fix_type = 0;
	}

	// Calculate UTC time since epoch
	double utc_time = rmc->timestamp;
	int utc_hour = static_cast<int>(utc_time / 10000);
	int utc_minute = static_cast<int>((utc_time - utc_hour * 10000) / 100);
	double utc_sec = static_cast<double>(utc_time - utc_hour * 10000 - utc_minute * 100);
	int nmea_day = static_cast<int>(rmc->date / 10000);
	int nmea_mth = static_cast<int>((rmc->date - nmea_day * 10000) / 100);
	int nmea_year = static_cast<int>(rmc->date - nmea_day * 10000 - nmea_mth * 100);
	// convert to unix timestamp
	struct tm timeinfo = {};
	timeinfo.tm_year = nmea_year + 100;
	timeinfo.tm_mon = nmea_mth - 1;
	timeinfo.tm_mday = nmea_day;
	timeinfo.tm_hour = utc_hour;
	timeinfo.tm_min = utc_minute;
	timeinfo.tm_sec = int(utc_sec);
	timeinfo.tm_isdst = 0;

	time_t epoch = mktime(&timeinfo);
	uint64_t usecs = static_cast<uint64_t>((utc_sec - static_cast<uint64_t>(utc_sec)) * 1000000);
	_gps_report.time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
	_gps_report.time_utc_usec += usecs;

	// Set clock if it hasn't been set or if the time has drifted too far
	if (!_clock_set) {
		timespec ts{};
		ts.tv_sec = epoch;
		ts.tv_nsec = usecs * 1000;
		PX4_INFO("Setting system clock");
		px4_clock_settime(CLOCK_REALTIME, &ts);
	}

	_gps_report.timestamp = hrt_absolute_time();
	_gps_report.device_id = get_device_id();

	_sensor_gps_pub.publish(_gps_report);

	jamming_spoofing_check();
}

void TeseoGPS::jamming_spoofing_check()
{
	if (_gps_report.spoofing_state != _spoofing_state) {

		if (_gps_report.spoofing_state > sensor_gps_s::SPOOFING_STATE_NONE) {
			PX4_WARN("GPS spoofing detected! (state: %d)", _gps_report.spoofing_state);
		}

		_spoofing_state = _gps_report.spoofing_state;
	}

	if (_gps_report.jamming_state != _jamming_state) {

		if (_gps_report.jamming_state > sensor_gps_s::JAMMING_STATE_WARNING) {
			PX4_WARN("GPS jamming detected! (state: %d) (indicator: %d)", _gps_report.jamming_state,
				 (uint8_t)_gps_report.jamming_indicator);
		}

		_jamming_state = _gps_report.jamming_state;
	}
}

int TeseoGPS::read_serial_port(uint8_t* buf, size_t size, int timeout_ms)
{
	const int max_timeout = 50;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	// https://man7.org/linux/man-pages/man2/poll.2.html
	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout_ms));

	if (ret <= 0) {
		// Timed out or error
		perf_count(_read_error);
		return ret;
	}

	// Check that there is data to read
	if (!(fds[0].revents & POLLIN)) {
		perf_count(_read_error);
		return -1;
	}

	/*
	 * We are here because poll says there is some data, so this
	 * won't block even on a blocking device. But don't read immediately
	 * by 1-2 bytes, wait for some more data to save expensive read() calls.
	 * If we have all requested data available, read it without waiting.
	 * If more bytes are available, we'll go back to poll() again.
	 */
	const unsigned character_count = 32; // minimum bytes that we want to read
	unsigned baudrate = _baudrate == 0 ? 115200 : _baudrate;
	const unsigned sleeptime = character_count * 1000000 / (baudrate / 10);

	int err = 0;
	int bytes_available = 0;
	err = ::ioctl(_serial_fd, FIONREAD, (unsigned long)&bytes_available);

	if (err != 0 || bytes_available < (int)character_count) {
		px4_usleep(sleeptime);
	}

	ret = ::read(_serial_fd, buf, size);

	// Return bytes read
	return ret;
}

int TeseoGPS::setBaudrate(unsigned baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

	return 0;
}

int TeseoGPS::print_status()
{
	perf_print_counter(_read_error);

	print_message(ORB_ID(sensor_gps), _gps_report);

	return 0;
}

int TeseoGPS::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	// TODO:

	return -1;
}

int TeseoGPS::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Teseo GPS driver module that handles the NMEA communication with the device and publishes the position via uORB.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("teseo_gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "GPS device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TeseoGPS::task_spawn(int argc, char *argv[])
{
	static constexpr int TASK_STACK_SIZE = PX4_STACK_ADJUSTED(2040);

	_task_id = px4_task_spawn_cmd("teseo_gps",
					SCHED_DEFAULT,
					SCHED_PRIORITY_SLOW_DRIVER,
					TASK_STACK_SIZE,
					&run_trampoline,
					(char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

TeseoGPS* TeseoGPS::instantiate(int argc, char *argv[])
{
	const char* device_path = nullptr;
	int baudrate = 0;

	int myoptind = 1;
	int ch;
	const char* myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				return nullptr;
			}
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized option: %c", ch);
			return nullptr;
		}
	}

	TeseoGPS* gps = nullptr;

	if (device_path && (::access(device_path, R_OK|W_OK) == 0)) {
		gps = new TeseoGPS(device_path, baudrate);

	} else {
		PX4_ERR("invalid device (-d) %s", device_path ? device_path  : "");
	}

	return gps;
}

extern "C" __EXPORT int teseo_gps_main(int argc, char *argv[])
{
	return TeseoGPS::main(argc, argv);
}