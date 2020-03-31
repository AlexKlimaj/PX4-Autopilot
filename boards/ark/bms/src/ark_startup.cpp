#include <board_config.h>
#include <lib/drivers/smbus/SMBus.hpp>

#define BATT_SMBUS_ADDR                                 0x0B
#define BATT_SMBUS_VOLTAGE                              0x09
#define BATT_SMBUS_CURRENT                              0x0A
#define BATT_SMBUS_RELATIVE_SOC							0x0D

static SMBus* _interface;

static uint16_t _voltage = 0;
static uint16_t _current = 0;
static uint16_t _soc = 0;

// Expose these functions for use in init.c
__BEGIN_DECLS
extern void bq40z80_startup_init(void);
__END_DECLS

void update_battery_data(void);

void bq40z80_startup_init(void)
{
	int address = BATT_SMBUS_ADDR;
	int bus = 1;

	// Create the SMBus interface and initialize it
	_interface = new SMBus(bus, address);
	_interface->init();

	// Create the SPI interface and initialize it


	// We need to get the voltage, current, and state of charge from the BQ
	update_battery_data();


	up_udelay(10000);


	// Do stuff with the display now
}

void update_battery_data(void)
{
		// Get the voltage
	_interface->read_word(BATT_SMBUS_VOLTAGE, _voltage);
	JAKE_DEBUG("voltage: %d", _voltage);

	// Get the current
	_interface->read_word(BATT_SMBUS_CURRENT, _current);
	JAKE_DEBUG("current: %d", _current);

	// Get the state of charge
	_interface->read_word(BATT_SMBUS_RELATIVE_SOC, _soc);
	JAKE_DEBUG("soc: %d", _soc);
}