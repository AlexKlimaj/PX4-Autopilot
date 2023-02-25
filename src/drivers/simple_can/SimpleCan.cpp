/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file SimpleCan.cpp
 *
 */

#include "SimpleCan.hpp"

extern orb_advert_t mavlink_log_pub;

SimpleCan::SimpleCan() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

SimpleCan::~SimpleCan()
{
}

void SimpleCan::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {

		_fd = ::open("/dev/can0", O_RDWR);

		if (_fd < 0) {
			PX4_INFO("FAILED TO OPEN /dev/can0");
			return;
		}

		_initialized = true;
	}
}

// int16_t SimpleCan::receive(CanFrame *received_frame)
// {
// 	if ((_fd < 0) || (received_frame == nullptr)) {
// 		PX4_INFO("fd < 0");
// 		return -1;
// 	}

// 	struct pollfd fds {};

// 	fds.fd = _fd;

// 	fds.events = POLLIN;

// 	// Jake: this doesn't block even in blocking mode, dunno if we need it
// 	::poll(&fds, 1, 0);

// 	// Only execute this part if can0 is changed.
// 	if (fds.revents & POLLIN) {

// 		// Try to read.
// 		struct can_msg_s receive_msg;
// 		const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));

// 		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
// 			PX4_INFO("error");
// 			return -1;

// 		} else {
// 			received_frame->extended_can_id = receive_msg.cm_hdr.ch_id;
// 			received_frame->payload_size = receive_msg.cm_hdr.ch_dlc;
// 			memcpy((void *)received_frame->payload, receive_msg.cm_data, receive_msg.cm_hdr.ch_dlc);
// 			return nbytes;
// 		}
// 	}

// 	return 0;
// }

int16_t SimpleCan::transmit(const CanFrame *txf, int timeout_ms)
{
	if (_fd < 0) {
		return -1;
	}

	struct pollfd fds {};

	fds.fd = _fd;

	fds.events |= POLLOUT;

	const int poll_result = poll(&fds, 1, timeout_ms);

	if (poll_result < 0) {
		return -1;
	}

	if (poll_result == 0) {
		return 0;
	}

	if ((fds.revents & POLLOUT) == 0) {
		return -1;
	}

	struct can_msg_s *transmit_msg;

	transmit_msg.cm_hdr.ch_id = txf.frame.extended_can_id;

	transmit_msg.cm_hdr.ch_dlc = txf.frame.payload_size;

	transmit_msg.cm_hdr.ch_extid = 1;

	memcpy(transmit_msg.cm_data, txf.frame.payload, txf.frame.payload_size);

	const size_t msg_len = CAN_MSGLEN(transmit_msg.cm_hdr.ch_dlc);

	const ssize_t nbytes = ::write(_fd, &transmit_msg, msg_len);

	if (nbytes < 0 || (size_t)nbytes != msg_len) {
		return -1;
	}

	return 1;
}


int SimpleCan::start()
{
	// There is a race condition at boot that sometimes causes opening of
	// /dev/can0 to fail. We will delay 0.5s to be safe.
	uint32_t delay_us = 500000;
	ScheduleOnInterval(1000000 / SAMPLE_RATE, delay_us);
	return PX4_OK;
}

int SimpleCan::task_spawn(int argc, char *argv[])
{
	SimpleCan *instance = new SimpleCan();

	if (!instance) {
		PX4_ERR("driver allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	instance->start();
	return 0;
}

int SimpleCan::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for sending simple can messages.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("simple_can", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int SimpleCan::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_INFO("not running");
		return PX4_ERROR;
	}

	return print_usage("Unrecognized command.");
}

extern "C" __EXPORT int simple_can_main(int argc, char *argv[])
{
	return SimpleCan::main(argc, argv);
}
