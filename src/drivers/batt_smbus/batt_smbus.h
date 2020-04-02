/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file batt_smbus.h
 *
 * Header for a battery monitor connected via SMBus (I2C).
 * Designed for BQ40Z50-R1/R2
 *
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 */

#pragma once

#include <ecl/geo/geo.h>
#include <lib/drivers/smbus/SMBus.hpp>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/battery_status.h>

#include <board_config.h>

#define MAC_DATA_BUFFER_SIZE                            32

#define BATT_CELL_VOLTAGE_THRESHOLD_RTL                 0.5f            ///< Threshold in volts to RTL if cells are imbalanced
#define BATT_CELL_VOLTAGE_THRESHOLD_FAILED              1.5f            ///< Threshold in volts to Land if cells are imbalanced

#define BATT_CURRENT_UNDERVOLTAGE_THRESHOLD             5.0f            ///< Threshold in amps to disable undervoltage protection
#define BATT_VOLTAGE_UNDERVOLTAGE_THRESHOLD             3.4f            ///< Threshold in volts to re-enable undervoltage protection

#define BATT_SMBUS_ADDR                                 0x0B            ///< Default 7 bit address I2C address. 8 bit = 0x16

#define BATT_SMBUS_CURRENT                              0x0A            ///< current register
#define BATT_SMBUS_AVERAGE_CURRENT                      0x0B            ///< average current register
#define BATT_SMBUS_MAX_ERROR				0x0C		///< max error
#define BATT_SMBUS_RELATIVE_SOC				0x0D		///< Relative State Of Charge
#define BATT_SMBUS_TEMP                                 0x08            ///< temperature register
#define BATT_SMBUS_TEMP                                 0x08            ///< temperature register
#define BATT_SMBUS_VOLTAGE                              0x09            ///< voltage register
#define BATT_SMBUS_FULL_CHARGE_CAPACITY                 0x10            ///< capacity when fully charged
#define BATT_SMBUS_RUN_TIME_TO_EMPTY                    0x11            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_AVERAGE_TIME_TO_EMPTY                0x12            ///< predicted remaining battery capacity based on the present rate of discharge in min
#define BATT_SMBUS_REMAINING_CAPACITY                   0x0F            ///< predicted remaining battery capacity as a percentage
#define BATT_SMBUS_CYCLE_COUNT                          0x17            ///< number of cycles the battery has experienced
#define BATT_SMBUS_DESIGN_CAPACITY                      0x18            ///< design capacity register
#define BATT_SMBUS_DESIGN_VOLTAGE                       0x19            ///< design voltage register
#define BATT_SMBUS_MANUFACTURER_NAME                    0x20            ///< manufacturer name
#define BATT_SMBUS_MANUFACTURE_DATE                     0x1B            ///< manufacture date register
#define BATT_SMBUS_SERIAL_NUMBER                        0x1C            ///< serial number register
#define BATT_SMBUS_MEASUREMENT_INTERVAL_US              100000          ///< time in microseconds, measure at 10Hz
#define BATT_SMBUS_MANUFACTURER_ACCESS                  0x00
#define BATT_SMBUS_MANUFACTURER_DATA                    0x23
#define BATT_SMBUS_MANUFACTURER_BLOCK_ACCESS            0x44
#define BATT_SMBUS_STATE_OF_HEALTH			0x4F		///< State of Health. The SOH information of the battery in percentage of Design Capacity
#define BATT_SMBUS_SECURITY_KEYS                        0x0035
#define BATT_SMBUS_CELL_4_VOLTAGE                       0x3F
#define BATT_SMBUS_CELL_5_VOLTAGE                       0x3E
#define BATT_SMBUS_CELL_6_VOLTAGE                       0x3D
#define BATT_SMBUS_LIFETIME_FLUSH                       0x002E
#define BATT_SMBUS_LIFETIME_BLOCK_ONE                   0x0060
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_ADDRESS        0x4938
#define BATT_SMBUS_SEAL                                 0x0030

#define BATT_SMBUS_ENABLED_PROTECTIONS_A_DEFAULT        0xcf
#define BATT_SMBUS_ENABLED_PROTECTIONS_A_CUV_DISABLED   0xce

// Dataflash addresses
#define BQ40Z80_SCALE_FACTOR_ADDR                       0x4AE8 ///< Address at which the Scale Factor is Stored. Accessed with MAC block read.
#define BQ40Z80_ENABLED_PROTECTIONS_A_ADDR              0x4BBE
#define BQ40Z80_ENABLED_PROTECTIONS_B_ADDR 	        0x4BBF
#define BQ40Z80_ENABLED_PROTECTIONS_C_ADDR              0x4BC0
#define BQ40Z80_ENABLED_PROTECTIONS_D_ADDR              0x4BC1

// Block Access commands
#define MAC_BA_DATA_BUFFER_SIZE                         32 // MAC buffer size
#define BQ40Z80_MAC_BA_HW_VER                           0x0003 ///<
#define BQ40Z80_MAC_BA_FET_EN                           0x0022 ///< This command disables/enables control of the CHG, DSG, and PCHG FET by the firmware
#define BQ40Z80_MAC_BA_DASTATUS1                        0x0071 ///< returns the cell voltages, pack voltage, bat voltage, cell currents, cell powers, power, and average power
#define BQ40Z80_MAC_BA_DASTATUS2                        0x0072 ///< returns the internal temperature sensor, TS1, TS2, TS3, TS4, Cell Temp, and FET Temp
#define BQ40Z80_MAC_BA_MANUFACTURING_STATUS             0x0057 ///< returns the ManufacturingStatus() flags on ManufacturerBlockAccess() or ManufacturerData().

// DASTATUS1 Info. See Page 130 of the BQ40Z80 Technical Reference Manual.
#define BQ40Z80_DASTATUS1_CELL1_VOLTAGE_MSB             1
#define BQ40Z80_DASTATUS1_CELL1_VOLTAGE_LSB             0
#define BQ40Z80_DASTATUS1_CELL2_VOLTAGE_MSB             3
#define BQ40Z80_DASTATUS1_CELL2_VOLTAGE_LSB             2
#define BQ40Z80_DASTATUS1_CELL3_VOLTAGE_MSB             5
#define BQ40Z80_DASTATUS1_CELL3_VOLTAGE_LSB             4
#define BQ40Z80_DASTATUS1_CELL4_VOLTAGE_MSB             7
#define BQ40Z80_DASTATUS1_CELL4_VOLTAGE_LSB             6
#define BQ40Z80_DASTATUS1_BAT_VOLTAGE_MSB               9
#define BQ40Z80_DASTATUS1_BAT_VOLTAGE_LSB               8
#define BQ40Z80_DASTATUS1_PACK_VOLTAGE_MSB              11
#define BQ40Z80_DASTATUS1_PACK_VOLTAGE_LSB              10

#define BQ40Z80_FET_EN_SHIFT 4
#define BQ40Z80_FET_EN_MASK (1 << BQ40Z80_FET_EN_SHIFT)

// Configurable Settings
#define BQ40Z80_SHUTDOWN_CURRENT_LIMIT_A 0.5f // Current charging or discharging must be less than this to allow turning off the FETs
#define BQ40Z80_PROTECTION_DISBABLE_CURRENT_THRESHOLD_A -0.4f // Discharge currents above this value will disable protections
#define BQ40Z80_ENABLED_PROTECTIONS_A_VAL 0xFF
#define BQ40Z80_ENABLED_PROTECTIONS_B_VAL 0x7F
#define BQ40Z80_ENABLED_PROTECTIONS_C_VAL 0x85
#define BQ40Z80_ENABLED_PROTECTIONS_D_VAL 0x0F

class BATT_SMBUS : public I2CSPIDriver<BATT_SMBUS>
{
public:
	BATT_SMBUS(I2CSPIBusOption bus_option, const int bus, SMBus *interface);

	~BATT_SMBUS();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	friend SMBus;

	void RunImpl();

	void custom_method(const BusCLIArguments &cli) override;

	/**
	 * @brief Configures the output FETS.
	 * @param enable Whether to enable or disable.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int configure_output_FETs(bool enable);

	/**
	 * @brief Configures the the protection settings.
	 * @param enable Whether to enable or disable.
	 * @param read_only True to only read state of protections.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int configure_protections(bool enable, bool read_only);

	/**
	 * @brief Writes data to flash.
	 * @param address The start address of the write.
	 * @param data The data to be written.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int dataflash_write(uint16_t address, void *data, const unsigned length);

	/**
	 * @brief Returns the SBS serial number of the battery device.
	 * @return Returns the SBS serial number of the battery device.
	 */
	uint16_t get_serial_number();

	/**
	* @brief Read info from battery on startup.
	* @return Returns PX4_OK on success, PX4_ERROR on failure.
	*/
	int get_startup_info();

	/**
	 * @brief Gets the SBS manufacture date of the battery.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacture_date();

	/**
	 * @brief Gets the SBS manufacturer name of the battery device.
	 * @param manufacturer_name Pointer to a buffer into which the manufacturer name is to be written.
	 * @param max_length The maximum number of bytes to attempt to read from the manufacturer name register,
	 *                   including the null character that is appended to the end.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_name(uint8_t *manufacturer_name, const uint8_t length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() read command.
	 * @param cmd_code The command code.
	 * @param data The returned data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_read(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Performs a ManufacturerBlockAccess() write command.
	 * @param cmd_code The command code.
	 * @param data The sent data.
	 * @param length The number of bytes being written.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int manufacturer_write(const uint16_t cmd_code, void *data, const unsigned length);

	/**
	 * @brief Unseals the battery to allow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int unseal();

	/**
	 * @brief Seals the battery to disallow writing to restricted flash.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int seal();

	/**
	 * @brief This command flushes the RAM Lifetime Data to data flash to help streamline evaluation testing.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_data_flush();

	/**
	 * @brief Reads the lifetime data from block 1.
	 * @return Returns PX4_OK on success, PX4_ERROR on failure.
	 */
	int lifetime_read_block_one();

	/**
	 * @brief Reads the cell voltages.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_voltages();

	/**
	 * @brief Reads the scale factor from teh BQ40Z80.
	 * @return Returns PX4_OK on success or associated read error code on failure.
	 */
	int get_scale_factor();

	void suspend();

	void resume();

private:

	SMBus *_interface;

	perf_counter_t _cycle{perf_alloc(PC_ELAPSED, "batt_smbus_cycle")};

	float _cell_voltages[6] {};

	float _max_cell_voltage_delta{0};

	float _min_cell_voltage{0};

	float _pack_voltage{0};

	float _bat_voltage{0};

	/** @param _last_report Last published report, used for test(). */
	battery_status_s _last_report{};

	/** @param _batt_topic uORB battery topic. */
	orb_advert_t _batt_topic{nullptr};

	/** @param _cell_count Number of series cell. */
	uint8_t _cell_count{4};

	/** @param _batt_capacity Battery design capacity in mAh (0 means unknown). */
	uint16_t _batt_capacity{0};

	/** @param _batt_startup_capacity Battery remaining capacity in mAh on startup. */
	uint16_t _batt_startup_capacity{0};

	/** @param _cycle_count The number of cycles the battery has experienced. */
	uint16_t _cycle_count{0};

	/** @param _max_error Expected margin of error, in %, in the state-of-charge calculation with a range of 1 to 100%. */
	uint16_t _max_error{0};

	/** @param _scale_factor Scale factor on teh BZ40Z80 */
	uint8_t _scale_factor{0};

	/** @param _serial_number Serial number register. */
	uint16_t _serial_number{0};

	/** @param _state_of_health State of health in %. */
	uint16_t _state_of_health{0};

	/** @param _crit_thr Critical battery threshold param. */
	float _crit_thr{0.f};

	/** @param _emergency_thr Emergency battery threshold param. */
	float _emergency_thr{0.f};

	/** @param _low_thr Low battery threshold param. */
	float _low_thr{0.f};

	/** @param _manufacturer_name Name of the battery manufacturer. */
	char *_manufacturer_name{nullptr};

	/** @param _lifetime_max_delta_cell_voltage Max lifetime delta of the battery cells */
	float _lifetime_max_delta_cell_voltage{0.f};

	/** @param _output_fets_enabled State of the output FETS */
	bool _output_fets_enabled{};

	/** @param _protections_enabled State of the protections */
	bool _protections_enabled{};

	/** @param _cell_undervoltage_protection_status */
	enum protection_status {
		NOT_PROTECTED = 0,
		PROTECTED = 1,
	};
	uint8_t _cell_undervoltage_protection_status {PROTECTED};


	BATT_SMBUS(const BATT_SMBUS &) = delete;
	BATT_SMBUS operator=(const BATT_SMBUS &) = delete;
};
