#include "UavcanCanard.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>

#include <cerrno>
#include <cstring>

// UAVCANv0 uavcan.protocol.NodeStatus
static constexpr uint16_t UAVCAN_NODE_STATUS_DTYPE_ID = 341;
static constexpr uint64_t UAVCAN_NODE_STATUS_SIGNATURE = 0x0f0868d0c1a7c6f1ULL;

// UAVCANv0 uavcan.protocol.GetNodeInfo
static constexpr uint16_t UAVCAN_GET_NODE_INFO_DTYPE_ID = 1;
static constexpr uint64_t UAVCAN_GET_NODE_INFO_SIGNATURE = 0xee468a8121c46a9eULL;

static constexpr const char *UAVCAN_NODE_NAME = "org.pixhawk.pixhawk";

class UavcanCanard::CanDev
{
public:
	virtual ~CanDev() = default;
	virtual int init(int32_t bitrate, uint8_t iface) = 0;
	virtual int16_t transmit(const CanardCANFrame &frame, int timeout_ms) = 0;
	virtual int16_t receive(CanardCANFrame *out_frame, uint64_t *out_timestamp_us) = 0;
};

#if defined(__PX4_NUTTX)

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

// NuttX CAN types (e.g. struct can_msg_s) depend on CONFIG_CAN.
#include <nuttx/config.h>

#if defined(CONFIG_CAN)
#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "stm32_can.h"
#endif

#include <px4_platform_common/board_common.h>

class CanDevNuttX : public UavcanCanard::CanDev
{
public:
	int init(int32_t bitrate, uint8_t iface) override
	{
		(void)bitrate;

		if (iface == 0) {
			PX4_ERR("invalid CAN iface");
			return -1;
		}

#if defined(CONFIG_CAN)
		// Ensure the CAN device exists (pattern used by the existing Cyphal NuttX backend).
		// Note: bitrate setup is BSP/driver-specific; for now we rely on board defaults.
		struct can_dev_s *can = stm32_caninitialize(iface);

		if (can == nullptr) {
			PX4_ERR("stm32_caninitialize failed");
			return -1;
		}

		char devpath[16] {};
		// Convention: CAN1->/dev/can0, CAN2->/dev/can1, ...
		snprintf(devpath, sizeof(devpath), "/dev/can%u", (unsigned)(iface - 1U));

		const int ret = can_register(devpath, can);

		if ((ret < 0) && (ret != -EEXIST)) {
			PX4_ERR("can_register %s failed: %d", devpath, ret);
			return -1;
		}

		_fd = ::open(devpath, O_RDWR | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("open %s failed (%d)", devpath, errno);
			return -1;
		}

		return 0;
#else
		return -ENOTSUP;
#endif
	}

	int16_t transmit(const CanardCANFrame &frame, int timeout_ms) override
	{
		if (_fd < 0) {
			return -1;
		}

#if !defined(CONFIG_CAN)
		(void)frame;
		(void)timeout_ms;
		return -1;
#else

		struct pollfd fds {};
		fds.fd = _fd;
		fds.events |= POLLOUT;

		const int poll_result = ::poll(&fds, 1, timeout_ms);

		if (poll_result <= 0) {
			return poll_result;
		}

		if ((fds.revents & POLLOUT) == 0) {
			return -1;
		}

		struct can_msg_s msg {};

		msg.cm_hdr.ch_id = frame.id & CANARD_CAN_EXT_ID_MASK;

		msg.cm_hdr.ch_extid = 1;

		msg.cm_hdr.ch_dlc = frame.data_len;

		memcpy(msg.cm_data, frame.data, frame.data_len);

		const size_t msg_len = CAN_MSGLEN(msg.cm_hdr.ch_dlc);

		const ssize_t nbytes = ::write(_fd, &msg, msg_len);

		if (nbytes < 0 || (size_t)nbytes != msg_len) {
			return -1;
		}

		return 1;
#endif
	}

	int16_t receive(CanardCANFrame *out_frame, uint64_t *out_timestamp_us) override
	{
		if ((_fd < 0) || (out_frame == nullptr)) {
			return -1;
		}

#if !defined(CONFIG_CAN)
		(void)out_timestamp_us;
		return -1;
#else

		struct pollfd fds {};
		fds.fd = _fd;
		fds.events = POLLIN;
		::poll(&fds, 1, 0);

		if ((fds.revents & POLLIN) == 0) {
			return 0;
		}

		struct can_msg_s msg {};

		const ssize_t nbytes = ::read(_fd, &msg, sizeof(msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(msg)) {
			return -1;
		}

		out_frame->id = (msg.cm_hdr.ch_id & CANARD_CAN_EXT_ID_MASK) | CANARD_CAN_FRAME_EFF;
		out_frame->data_len = msg.cm_hdr.ch_dlc;
		out_frame->iface_id = 0;
		memcpy(out_frame->data, msg.cm_data, msg.cm_hdr.ch_dlc);

		if (out_timestamp_us) {
			*out_timestamp_us = hrt_absolute_time();
		}

		return (int16_t)nbytes;

#endif
	}

private:
	int _fd{-1};
};

#endif // __PX4_NUTTX

UavcanCanard::UavcanCanard() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
	_param_uavcan_enable = param_find("UAVCAN_ENABLE");
	_param_uavcan_bitrate = param_find("UAVCAN_BITRATE");
	_param_uavcan_node_id = param_find("UAVCAN_NODE_ID");
}

UavcanCanard::~UavcanCanard()
{
	_should_exit.store(true);
	ScheduleNow();

	delete _can;
	_can = nullptr;
}

int UavcanCanard::start()
{
	// Delay a bit to avoid early-boot CAN device races (matches other CAN drivers in PX4).
	ScheduleOnInterval(10000, 500000);
	return PX4_OK;
}

bool UavcanCanard::init_canard()
{
	if (_initialized.load()) {
		return true;
	}

	// Parameters might not be present on some builds. Avoid param_get() on invalid handles.
	if (_param_uavcan_enable != PARAM_INVALID) {
		param_get(_param_uavcan_enable, &_uavcan_enable);
	} else {
		_uavcan_enable = 0;
	}

	if (_param_uavcan_bitrate != PARAM_INVALID) {
		param_get(_param_uavcan_bitrate, &_uavcan_bitrate);
	} else {
		_uavcan_bitrate = 1000000;
	}

	if (_param_uavcan_node_id != PARAM_INVALID) {
		param_get(_param_uavcan_node_id, &_uavcan_node_id);
	} else {
		_uavcan_node_id = 1;
	}

	// Legacy parity: UAVCAN_ENABLE selects the CAN interface.
	// - 1: CAN1 (/dev/can0)
	// - 2: CAN2 (/dev/can1)
	// - >2: multiple interfaces requested (not supported yet by this driver)
	if (!_can_iface_set) {
		_can_iface = (_uavcan_enable == 2) ? 2U : 1U;

		if (_uavcan_enable > 2) {
			PX4_WARN("UAVCAN_ENABLE=%ld requests multiple interfaces; using CAN%u", (long)_uavcan_enable, (unsigned)_can_iface);
		}
	}

	if (_uavcan_node_id <= 0 || _uavcan_node_id > 127) {
		PX4_ERR("invalid UAVCAN_NODE_ID=%ld", (long)_uavcan_node_id);
		return false;
	}

#if defined(__PX4_NUTTX)
	_can = new CanDevNuttX();

	int can_init_ret = -1;

	if (_can) {
		can_init_ret = _can->init(_uavcan_bitrate, _can_iface);
	}

	if (can_init_ret != 0) {
		if (can_init_ret == -ENOTSUP) {
			PX4_ERR("CAN support not built into firmware (enable NuttX CAN: CONFIG_CAN)");
			_should_exit.store(true);
			delete _can;
			_can = nullptr;
			return false;
		}

		PX4_ERR("CAN init failed");
		delete _can;
		_can = nullptr;
		return false;
	}

	px4_guid_t px4_guid{};

	if (board_get_px4_guid(px4_guid) > 0) {
		memcpy(_unique_id, &px4_guid[2], sizeof(_unique_id));
		_unique_id_valid = true;
	}

#else
	PX4_ERR("uavcan_canard is only supported on NuttX");
	return false;
#endif

	canardInit(&_canard,
		   _arena,
		   sizeof(_arena),
		   on_transfer_reception,
		   should_accept_transfer,
		   this);

	canardSetLocalNodeID(&_canard, (uint8_t)_uavcan_node_id);

	_last_1hz_us = 0;
	_last_cleanup_us = 0;
	_initialized.store(true);

	PX4_INFO("libcanard node started: node_id=%ld bitrate=%ld", (long)_uavcan_node_id, (long)_uavcan_bitrate);
	return true;
}

void UavcanCanard::handle_get_node_info(CanardInstance *ins, CanardRxTransfer *transfer)
{
	// Response payload is byte-aligned; the final dynamic array (name) uses TAO (libcanard default for CAN2.0).
	uint8_t payload[256] {};

	uint32_t offset = 0;

	// uavcan.protocol.NodeStatus (56 bits)
	const uint32_t uptime_sec = (uint32_t)(hrt_absolute_time() / 1000000ULL);
	const uint8_t health = 0; // HEALTH_OK
	const uint8_t mode = 0;   // MODE_OPERATIONAL
	const uint8_t sub_mode = 0;
	const uint16_t vendor_code = 0;

	canardEncodeScalar(payload, offset, 32, &uptime_sec);
	offset += 32;
	canardEncodeScalar(payload, offset, 2, &health);
	offset += 2;
	canardEncodeScalar(payload, offset, 3, &mode);
	offset += 3;
	canardEncodeScalar(payload, offset, 3, &sub_mode);
	offset += 3;
	canardEncodeScalar(payload, offset, 16, &vendor_code);
	offset += 16;

	// uavcan.protocol.SoftwareVersion (byte-aligned)
	const uint8_t sw_major = 0;
	const uint8_t sw_minor = 0;
	const uint8_t sw_optional_flags = 0;
	const uint32_t sw_vcs_commit = 0;
	const uint64_t sw_image_crc = 0;

	canardEncodeScalar(payload, offset, 8, &sw_major);
	offset += 8;
	canardEncodeScalar(payload, offset, 8, &sw_minor);
	offset += 8;
	canardEncodeScalar(payload, offset, 8, &sw_optional_flags);
	offset += 8;
	canardEncodeScalar(payload, offset, 32, &sw_vcs_commit);
	offset += 32;
	canardEncodeScalar(payload, offset, 64, &sw_image_crc);
	offset += 64;

	// uavcan.protocol.HardwareVersion (byte-aligned)
	const uint8_t hw_major = 0;
	const uint8_t hw_minor = 0;
	canardEncodeScalar(payload, offset, 8, &hw_major);
	offset += 8;
	canardEncodeScalar(payload, offset, 8, &hw_minor);
	offset += 8;

	for (unsigned i = 0; i < sizeof(_unique_id); i++) {
		const uint8_t b = _unique_id_valid ? _unique_id[i] : 0;
		canardEncodeScalar(payload, offset, 8, &b);
		offset += 8;
	}

	// certificate_of_authenticity: uint8[<=255] (not last field of outer structure, so explicit length)
	const uint8_t coa_len = 0;
	canardEncodeScalar(payload, offset, 8, &coa_len);
	offset += 8;

	// name: uint8[<=80] (last field => TAO: no length field, just bytes)
	const char *name = UAVCAN_NODE_NAME;
	const size_t name_len = strnlen(name, 80);

	for (size_t i = 0; i < name_len; i++) {
		const uint8_t ch = (uint8_t)name[i];
		canardEncodeScalar(payload, offset, 8, &ch);
		offset += 8;
	}

	const uint16_t payload_len = (uint16_t)((offset + 7U) / 8U);

	(void)canardRequestOrRespond(ins,
				     transfer->source_node_id,
				     UAVCAN_GET_NODE_INFO_SIGNATURE,
				     (uint8_t)UAVCAN_GET_NODE_INFO_DTYPE_ID,
				     &transfer->transfer_id,
				     transfer->priority,
				     CanardResponse,
				     payload,
				     payload_len);
}

void UavcanCanard::publish_node_status()
{
	uint8_t payload[7] {};

	const uint32_t uptime_sec = (uint32_t)(hrt_absolute_time() / 1000000ULL);
	const uint8_t health = 0; // HEALTH_OK
	const uint8_t mode = 0;   // MODE_OPERATIONAL
	const uint8_t sub_mode = 0;
	const uint16_t vendor_code = 0;

	uint32_t offset = 0;
	canardEncodeScalar(payload, offset, 32, &uptime_sec);
	offset += 32;
	canardEncodeScalar(payload, offset, 2, &health);
	offset += 2;
	canardEncodeScalar(payload, offset, 3, &mode);
	offset += 3;
	canardEncodeScalar(payload, offset, 3, &sub_mode);
	offset += 3;
	canardEncodeScalar(payload, offset, 16, &vendor_code);

	(void)canardBroadcast(&_canard,
			      UAVCAN_NODE_STATUS_SIGNATURE,
			      UAVCAN_NODE_STATUS_DTYPE_ID,
			      &_node_status_tid,
			      CANARD_TRANSFER_PRIORITY_LOW,
			      payload,
			      sizeof(payload));
}

void UavcanCanard::process_tx_queue()
{
	if (!_can) {
		return;
	}

	while (true) {
		CanardCANFrame *txf = canardPeekTxQueue(&_canard);

		if (txf == nullptr) {
			break;
		}

		(void)_can->transmit(*txf, 0);
		canardPopTxQueue(&_canard);
	}
}

void UavcanCanard::Run()
{
	if (_should_exit.load()) {
		ScheduleClear();
		return;
	}

	if (!_initialized.load()) {
		if (!init_canard()) {
			// Retry later.
			return;
		}
	}

	// RX processing (future parity work) - currently not accepting transfers.
	if (_can) {
		CanardCANFrame rxf{};
		uint64_t ts_us{};

		while (_can->receive(&rxf, &ts_us) > 0) {
			(void)canardHandleRxFrame(&_canard, &rxf, ts_us);
		}
	}

	const uint64_t now = hrt_absolute_time();

	if ((_last_1hz_us == 0) || (now - _last_1hz_us >= 1000000ULL)) {
		_last_1hz_us = now;
		publish_node_status();
	}

	if ((_last_cleanup_us == 0) || (now - _last_cleanup_us >= CANARD_RECOMMENDED_STALE_TRANSFER_CLEANUP_INTERVAL_USEC)) {
		_last_cleanup_us = now;
		canardCleanupStaleTransfers(&_canard, now);
	}

	process_tx_queue();
}

void UavcanCanard::on_transfer_reception(CanardInstance *ins, CanardRxTransfer *transfer)
{
	UavcanCanard *self = static_cast<UavcanCanard *>(canardGetUserReference(ins));

	if (!self || !transfer) {
		return;
	}

	if ((transfer->transfer_type == CanardTransferTypeRequest) && (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DTYPE_ID)) {
		// Recommended by libcanard: release RX buffers before enqueuing new TX transfers.
		canardReleaseRxTransferPayload(ins, transfer);
		self->handle_get_node_info(ins, transfer);
		return;
	}
}

bool UavcanCanard::should_accept_transfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature,
		uint16_t data_type_id,
		CanardTransferType transfer_type,
		uint8_t source_node_id)
{
	(void)ins;
	(void)source_node_id;

	if (transfer_type == CanardTransferTypeRequest) {
		switch (data_type_id) {
		case UAVCAN_GET_NODE_INFO_DTYPE_ID:
			*out_data_type_signature = UAVCAN_GET_NODE_INFO_SIGNATURE;
			return true;

		default:
			break;
		}
	}

	return false;
}

int UavcanCanard::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch = 0;

	bool set_enable = false;
	bool set_bitrate = false;
	bool set_node_id = false;
	bool set_iface = false;

	int32_t enable = 1;
	int32_t bitrate = 0;
	int32_t node_id = 0;
	int32_t iface = 1;

	while ((ch = px4_getopt(argc, argv, "hei:n:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'h':
			return print_usage();

		case 'e':
			set_enable = true;
			break;

		case 'i':
			iface = (int32_t)strtol(myoptarg, nullptr, 10);
			set_iface = true;
			break;

		case 'n':
			node_id = (int32_t)strtol(myoptarg, nullptr, 10);
			set_node_id = true;
			break;

		case 'b':
			bitrate = (int32_t)strtol(myoptarg, nullptr, 10);
			set_bitrate = true;
			break;

		default:
			return print_usage("Invalid argument");
		}
	}

	// Apply CLI overrides via parameters so behavior stays consistent.
	if (set_enable) {
		const param_t p = param_find("UAVCAN_ENABLE");

		if (p != PARAM_INVALID) {
			(void)param_set(p, &enable);
		}
	}

	if (set_node_id) {
		if (node_id <= 0 || node_id > 127) {
			PX4_ERR("invalid node id %ld", (long)node_id);
			return PX4_ERROR;
		}

		const param_t p = param_find("UAVCAN_NODE_ID");

		if (p != PARAM_INVALID) {
			(void)param_set(p, &node_id);
		}
	}

	if (set_bitrate) {
		if (bitrate <= 0) {
			PX4_ERR("invalid bitrate %ld", (long)bitrate);
			return PX4_ERROR;
		}

		const param_t p = param_find("UAVCAN_BITRATE");

		if (p != PARAM_INVALID) {
			(void)param_set(p, &bitrate);
		}
	}

	if (set_iface) {
		if (iface <= 0 || iface > 4) {
			PX4_ERR("invalid iface %ld", (long)iface);
			return PX4_ERROR;
		}
	}

	UavcanCanard *instance = new UavcanCanard();

	if (!instance) {
		PX4_ERR("allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (set_iface) {
		instance->_can_iface = (uint8_t)iface;
		instance->_can_iface_set = true;
	}

	instance->start();
	return PX4_OK;
}

int UavcanCanard::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

int UavcanCanard::print_usage(const char *reason)
{
	if (reason) {
		PX4_INFO("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
DroneCAN/UAVCANv0 driver based on DroneCAN libcanard.

This driver is under active development and is intended to become feature-parity
compatible with the existing libdronecan/libuavcan-based driver.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uavcan_canard", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('e', "Enable UAVCAN (sets UAVCAN_ENABLE=1)", true);
	PRINT_MODULE_USAGE_PARAM_INT('i', 1, 1, 4, "CAN interface (1->/dev/can0, 2->/dev/can1)", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', -1, 1, 127, "Node ID (sets UAVCAN_NODE_ID)", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 1000000, 10000, 8000000, "CAN bitrate (sets UAVCAN_BITRATE)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
