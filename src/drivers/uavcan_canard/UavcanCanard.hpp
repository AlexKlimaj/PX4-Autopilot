#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <px4_platform_common/atomic.h>

#include <cstdint>

extern "C" {
#include <canard.h>
}

class UavcanCanard : public ModuleBase<UavcanCanard>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	UavcanCanard();
	~UavcanCanard() override;

	// Minimal CAN device backend (platform-specific implementation in .cpp)
	class CanDev;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int start();

private:
	void Run() override;

	bool init_canard();
	void publish_node_status();
	void process_tx_queue();
	void handle_get_node_info(CanardInstance *ins, CanardRxTransfer *transfer);

	// libcanard callbacks
	static void on_transfer_reception(CanardInstance *ins, CanardRxTransfer *transfer);
	static bool should_accept_transfer(const CanardInstance *ins,
					   uint64_t *out_data_type_signature,
					   uint16_t data_type_id,
					   CanardTransferType transfer_type,
					   uint8_t source_node_id);

	// Parameters (must match existing PX4-facing behavior)
	param_t _param_uavcan_enable{PARAM_INVALID};
	param_t _param_uavcan_bitrate{PARAM_INVALID};
	param_t _param_uavcan_node_id{PARAM_INVALID};

	int32_t _uavcan_enable{0};
	int32_t _uavcan_bitrate{1000000};
	int32_t _uavcan_node_id{0};
	uint8_t _can_iface{1};
	bool _can_iface_set{false};

	px4::atomic_bool _initialized{false};
	px4::atomic_bool _should_exit{false};

	CanardInstance _canard{};
	uint8_t _node_status_tid{0};
	uint64_t _last_1hz_us{0};
	uint64_t _last_cleanup_us{0};

	// Arena for libcanard dynamic allocations.
	static constexpr size_t ArenaSize = 8192;
	alignas(8) uint8_t _arena[ArenaSize] {};

	// Used for GetNodeInfo/allocations.
	uint8_t _unique_id[16] {};
	bool _unique_id_valid{false};

	// Minimal CAN device (NuttX only)
	CanDev *_can{nullptr};
};
