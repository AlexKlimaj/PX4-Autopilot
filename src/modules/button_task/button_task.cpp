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
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>

#define BUTTON_DEBOUNCE_DURATION_US    1000
#define BUTTON_SHORT_PRESS_DURATION_US 300000
#define BUTTON_LONG_PRESS_DURATION_US  500000
#define BQ40Z80_SHUTDOWN_CURRENT_LIMIT_A 0.5f // Current charging or discharging must be less than this to allow turning off the FETs
static constexpr uint64_t BUTTON_SHUTDOWN_DURATION_US = 3000000ul;

using namespace time_literals;

static bool button_pressed = false;

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

	void updateLEDs(const battery_status_s& data);
	void pulseLEDs(void);

	uORB::Subscription _battery_sub{ORB_ID(battery_status)};

	int _led_pulse_state = {};
};

ButtonTask::ButtonTask() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// Anything to do?
}

ButtonTask::~ButtonTask()
{
	ScheduleClear();
}

int ButtonTask::isr_callback(int irq, FAR void *context, void *arg)
{
	printf("button pressed: %d\n", irq);
	button_pressed = true;
	return OK;
}

void
ButtonTask::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	battery_status_s data;
	bool battery_update = _battery_sub.update(&data);

	if (!button_pressed) {
		// Update LEDs with battery state
		if (battery_update) {
			updateLEDs(data);
		}
		return;
	} else {
		button_pressed = false;
	}

	bool state = stm32_gpioread(GPIO_BUTTON);

	auto start_time = hrt_absolute_time();
	auto time_now = start_time;

	while (!state) {
		time_now = hrt_absolute_time();

		uint64_t elapsed = time_now - start_time;
		PX4_INFO("button held: %d", (int)elapsed);

		// Pulse the LEDs
		pulseLEDs();

		if ((elapsed > BUTTON_SHUTDOWN_DURATION_US) && (data.current_a < BQ40Z80_SHUTDOWN_CURRENT_LIMIT_A)) {

			// TODO: emit shutdown message
			PX4_INFO("Time to shut down");

			// Time to shutdown
			px4_arch_gpiosetevent(GPIO_BUTTON, false, true, false, &ButtonTask::isr_callback, this);

			stm32_gpiowrite(GPIO_PWR_EN, false);
		}

		usleep(100000); // 10hz
		state = stm32_gpioread(GPIO_BUTTON);
	}

	_led_pulse_state = 0;
}

void ButtonTask::updateLEDs(const battery_status_s& data)
{
	float remaining = data.remaining; // 0 - 1
	int remaining_fifth = (int)(remaining * 5 + 1);

	switch (remaining_fifth)
	{
		case 1:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, true);
			stm32_gpiowrite(GPIO_LED_3, true);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;
		case 2:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, false);
			stm32_gpiowrite(GPIO_LED_3, true);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;
		case 3:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, false);
			stm32_gpiowrite(GPIO_LED_3, false);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;
		case 4:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, false);
			stm32_gpiowrite(GPIO_LED_3, false);
			stm32_gpiowrite(GPIO_LED_2, false);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;
		case 5:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, false);
			stm32_gpiowrite(GPIO_LED_3, false);
			stm32_gpiowrite(GPIO_LED_2, false);
			stm32_gpiowrite(GPIO_LED_1, false);
			break;
	}
}

void ButtonTask::pulseLEDs(void)
{
	switch (_led_pulse_state)
	{
		case 0:
			stm32_gpiowrite(GPIO_LED_5, true);
			stm32_gpiowrite(GPIO_LED_4, true);
			stm32_gpiowrite(GPIO_LED_3, true);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;

		case 1:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, true);
			stm32_gpiowrite(GPIO_LED_3, true);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;

		case 2:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, false);
			stm32_gpiowrite(GPIO_LED_3, true);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;

		case 3:
			stm32_gpiowrite(GPIO_LED_5, false);
			stm32_gpiowrite(GPIO_LED_4, false);
			stm32_gpiowrite(GPIO_LED_3, false);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;

		case 4:
			stm32_gpiowrite(GPIO_LED_5, true);
			stm32_gpiowrite(GPIO_LED_4, false);
			stm32_gpiowrite(GPIO_LED_3, false);
			stm32_gpiowrite(GPIO_LED_2, false);
			stm32_gpiowrite(GPIO_LED_1, true);
			break;

		case 5:
			stm32_gpiowrite(GPIO_LED_5, true);
			stm32_gpiowrite(GPIO_LED_4, true);
			stm32_gpiowrite(GPIO_LED_3, false);
			stm32_gpiowrite(GPIO_LED_2, false);
			stm32_gpiowrite(GPIO_LED_1, false);
			break;

		case 6:
			stm32_gpiowrite(GPIO_LED_5, true);
			stm32_gpiowrite(GPIO_LED_4, true);
			stm32_gpiowrite(GPIO_LED_3, true);
			stm32_gpiowrite(GPIO_LED_2, false);
			stm32_gpiowrite(GPIO_LED_1, false);
			break;

		case 7:
			stm32_gpiowrite(GPIO_LED_5, true);
			stm32_gpiowrite(GPIO_LED_4, true);
			stm32_gpiowrite(GPIO_LED_3, true);
			stm32_gpiowrite(GPIO_LED_2, true);
			stm32_gpiowrite(GPIO_LED_1, false);
			break;
	}

	_led_pulse_state++;

	if (_led_pulse_state == 8) {
		_led_pulse_state = 0;
	}
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

	// Enable button interrupts for shutdown
	px4_arch_configgpio(GPIO_BUTTON);
	px4_arch_gpiosetevent(GPIO_BUTTON, false, true, true, &ButtonTask::isr_callback, this);

	PX4_INFO("starting");

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
