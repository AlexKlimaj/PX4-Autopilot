#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <cstdint>

#include <uORB/Subscription.hpp>

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND) || defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)
# include <uORB/Publication.hpp>
#endif

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND)
# include <uORB/topics/tune_control.h>
#endif

#if defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)
# include <uORB/topics/led_control.h>
#endif

extern "C" {
#include <canard.h>
}

class UavcannodeCanard : public px4::ScheduledWorkItem
{
public:
	static int start(int32_t node_id, int32_t bitrate);
	static UavcannodeCanard *instance() { return _instance; }

	~UavcannodeCanard() override;

	void PrintInfo();

private:
	UavcannodeCanard(int32_t node_id, int32_t bitrate);

	void Run() override;
	bool init_canard();

	void process_tx_queue();
	void process_rx();

	void publish_node_status();
	void handle_get_node_info(CanardInstance *ins, CanardRxTransfer *transfer);
	void handle_allocation_message(CanardRxTransfer *transfer);
	void allocation_send_request(bool first_part, uint8_t unique_id_offset);

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND)
	void handle_beep_command(const CanardRxTransfer *transfer);
#endif

#if defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)
	void handle_lights_command(const CanardRxTransfer *transfer);
#endif

	// libcanard callbacks
	static void on_transfer_reception(CanardInstance *ins, CanardRxTransfer *transfer);
	static bool should_accept_transfer(const CanardInstance *ins,
					   uint64_t *out_data_type_signature,
					   uint16_t data_type_id,
					   CanardTransferType transfer_type,
					   uint8_t source_node_id);

	class CanDev;
	CanDev *_can{nullptr};

	px4::atomic_bool _initialized{false};
	px4::atomic_bool _should_exit{false};

	int32_t _bitrate{1000000};
	int32_t _configured_node_id{0};

	CanardInstance _canard{};
	uint8_t _node_status_tid{0};
	uint8_t _alloc_tid{0};
	uint64_t _last_1hz_us{0};
	uint64_t _last_cleanup_us{0};

	// Allocation state
	uint64_t _alloc_next_request_us{0};
	uint64_t _alloc_followup_until_us{0};
	uint8_t _alloc_unique_id_offset{0};
	bool _alloc_followup_pending{false};

	uint8_t _unique_id[16] {};
	bool _unique_id_valid{false};

#if defined(CONFIG_UAVCANNODE_FLOW_MEASUREMENT)
	uORB::Subscription _flow_sub {ORB_ID(vehicle_optical_flow)};
	uint8_t _flow_tid{0};
#endif

#if defined(CONFIG_UAVCANNODE_RANGE_SENSOR_MEASUREMENT)
	uORB::Subscription _distance_sensor_sub {ORB_ID(distance_sensor)};
	uint8_t _range_tid{0};
#endif

#if defined(CONFIG_UAVCANNODE_RAW_IMU)
	uORB::Subscription _vehicle_imu_sub {ORB_ID(vehicle_imu)};
	uint8_t _raw_imu_tid{0};
	bool _publish_raw_imu{false};
#endif

#if defined(CONFIG_UAVCANNODE_GNSS_FIX)
	uORB::Subscription _gps_sub {ORB_ID(sensor_gps)};
	uint8_t _gnss_fix2_tid{0};
	uint8_t _gnss_aux_tid{0};
#endif

#if defined(CONFIG_UAVCANNODE_MAGNETIC_FIELD_STRENGTH)
	uORB::Subscription _mag_sub {ORB_ID(vehicle_magnetometer)};
	uint8_t _mag_tid{0};
#endif

#if defined(CONFIG_UAVCANNODE_STATIC_PRESSURE) || defined(CONFIG_UAVCANNODE_STATIC_TEMPERATURE)
	uORB::Subscription _baro_sub {ORB_ID(sensor_baro)};
#endif

#if defined(CONFIG_UAVCANNODE_STATIC_PRESSURE)
	uint8_t _static_pressure_tid {0};
#endif

#if defined(CONFIG_UAVCANNODE_STATIC_TEMPERATURE)
	uint8_t _static_temperature_tid {0};
	uint64_t _last_static_temperature_pub_us{0};
#endif

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND)
	uORB::Publication<tune_control_s> _tune_control_pub {ORB_ID(tune_control)};
#endif

#if defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)
	uORB::Publication<led_control_s> _led_control_pub {ORB_ID(led_control)};
	unsigned _self_light_index{0};
#endif

#if defined(CONFIG_UAVCANNODE_SAFETY_BUTTON)
	uORB::Subscription _safety_button_sub {ORB_ID(safety_button)};
	uint8_t _safety_button_tid{0};
#endif

	static constexpr size_t ArenaSize = 8192;
	alignas(8) uint8_t _arena[ArenaSize] {};

	static UavcannodeCanard *_instance;
};
