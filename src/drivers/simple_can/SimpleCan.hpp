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

#pragma once

#include <fcntl.h>
#include <poll.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>

#include <px4_platform_common/px4_config.h>

#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

typedef struct {
	uint64_t timestamp_usec;
	uint32_t extended_can_id;
	size_t      payload_size;
	const void *payload;
} CanFrame;

class SimpleCan : public ModuleBase<SimpleCan>, public px4::ScheduledWorkItem
{
public:
	SimpleCan();

	virtual ~SimpleCan();

	static int print_usage(const char *reason = nullptr);
	static int custom_command(int argc, char *argv[]);

	static int task_spawn(int argc, char *argv[]);

	int start();

	//int16_t receive(CanFrame *received_frame);
	int transmit(const CanFrame &frame);

private:
	static constexpr uint32_t SAMPLE_RATE{1000}; // samples per second (1ms)
	static constexpr size_t TAIL_BYTE_START_OF_TRANSFER{128};

	void Run() override;

	int _fd{-1};
	bool _initialized{false};

	uORB::Subscription	_distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription	_sensor_optical_flow_sub{ORB_ID(sensor_optical_flow)};
};
