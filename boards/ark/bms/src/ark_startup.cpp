#include <board_config.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include "ssd1306.h"
#include <uORB/topics/battery_status.h>

#define BATT_SMBUS_ADDR                                 0x0B
#define BATT_SMBUS_VOLTAGE                              0x09
#define BATT_SMBUS_CURRENT                              0x0A
#define BATT_SMBUS_RELATIVE_SOC							0x0D

static SMBus* _interface;

static float _voltage_mv = 0;
static float _current_ma = 0;
static float _soc = 0;

// Expose these functions for use in init.c
__BEGIN_DECLS
extern void display_bq_startup_init(void);
extern void check_button_and_update_display();
__END_DECLS

void update_battery_data(void);
void update_led_status(void);



void display_bq_startup_init(void)
{
	int address = BATT_SMBUS_ADDR;
	int bus = 1;

	// Create the SMBus interface and initialize it
	_interface = new SMBus(bus, address);
	_interface->init();

	// Create the SPI interface and initialize it
	ssd1306::init();

	up_udelay(10000);
}

void check_button_and_update_display()
{
	unsigned button_held_count = 0;
	unsigned counter = 0;

	// Button should be held for 3 seconds
	while ((counter < 30)) {
		update_battery_data();
		ssd1306::updateStatus(_voltage_mv, _current_ma, _soc);
		update_led_status();

		counter++;
		up_udelay(50000); // 100ms

		bool button_held = !stm32_gpioread(GPIO_BUTTON);

		if (button_held) {
			printf("Keep holding that button big guy...\n");
			button_held_count++;
		}
	}

	if (button_held_count != counter) {
		printf("Button not held, powering off\n");

		// Drive the power enable low to power off system
		counter = 0;
		stm32_gpiowrite(GPIO_PWR_EN, false);
		up_udelay(1000000); // 1s
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

void update_battery_data(void)
{
	uint16_t result = 0;

	// Get the voltage
	_interface->read_word(BATT_SMBUS_VOLTAGE, result);
	_voltage_mv = ((float)result) / 1000.0f;

	// Get the current
	_interface->read_word(BATT_SMBUS_CURRENT, result);
	_current_ma = result;

	// Get the state of charge
	_interface->read_word(BATT_SMBUS_RELATIVE_SOC, result);
	_soc = ((float)result) / 100.0f;
}

void update_led_status(void)
{
	float remaining = _soc; // 0 - 1
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