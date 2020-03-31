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
 * @file sensors.cpp
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/battery/battery.h>
#include <lib/conversion/rotation.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/adc_report.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

using namespace time_literals;

static px4_sem_t _semaphore = {};

class ButtonTask : public ModuleBase<ButtonTask>, public px4::ScheduledWorkItem
{
public:
	ButtonTask();
	~ButtonTask() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	static int isr_callback(int irq, FAR void *context, void *arg);

private:
	void Run() override;

};

ButtonTask::ButtonTask() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Anything to do?
}

ButtonTask::~ButtonTask()
{
	ScheduleClear();

	px4_sem_destroy(&_semaphore);
}

int ButtonTask::isr_callback(int irq, FAR void *context, void *arg)
{
	printf("button pressed: %d\n", irq);
	px4_sem_post(&_semaphore);

	return OK;
}

void
ButtonTask::Run()
{
	px4_sem_wait(&_semaphore);

	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	PX4_INFO("button task running...");
}

int
ButtonTask::task_spawn(int argc, char *argv[])
{
	ButtonTask *instance = new ButtonTask();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool
ButtonTask::init()
{
	ScheduleOnInterval(100_ms);

	px4_sem_init(&_semaphore, 0, 0);

	// Enable button interrupts for shutdown
	px4_arch_configgpio(GPIO_BUTTON);
	px4_arch_gpiosetevent(GPIO_BUTTON, false, true, true, &ButtonTask::isr_callback, this);

	return true;
}

int ButtonTask::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ButtonTask::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

When the button is held, this task will monitor it and shut the system down if held for more than 3 seconds.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("button_task", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int button_task_main(int argc, char *argv[])
{
	return ButtonTask::main(argc, argv);
}
