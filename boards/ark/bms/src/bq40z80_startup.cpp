#include <board_config.h>
#include <lib/drivers/smbus/SMBus.hpp>

#define BATT_SMBUS_ADDR                                 0x0B
#define BATT_SMBUS_VOLTAGE                              0x09

SMBus* _interface;

// Expose these functions for use in init.c
__BEGIN_DECLS
extern void bq40z80_startup_init(void);
__END_DECLS

void bq40z80_startup_init(void)
{
	int address = BATT_SMBUS_ADDR;
	int bus = 1;

	_interface = new SMBus(bus, address);
	JAKE_DEBUG("Trying init");
	_interface->init();

	JAKE_DEBUG("Trying a read word");
	uint16_t voltage = 11;
	int result = _interface->read_word(BATT_SMBUS_VOLTAGE, voltage);

	JAKE_DEBUG("result: %d", result);
	JAKE_DEBUG("voltage: %d", voltage);
}