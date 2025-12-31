#include "UavcannodeCanard.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/micro_hal.h>

#include <drivers/drv_watchdog.h>

#include <parameters/param.h>

#include <cstring>

#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
extern "C" bool board_booted_by_px4(void);
#endif

#if defined(__PX4_NUTTX)
#include <drivers/bootloaders/boot_app_shared.h>

#ifndef APP_VERSION_MAJOR
#define APP_VERSION_MAJOR 0
#endif
#ifndef APP_VERSION_MINOR
#define APP_VERSION_MINOR 0
#endif
#ifndef HW_VERSION_MAJOR
#define HW_VERSION_MAJOR 0
#endif
#ifndef HW_VERSION_MINOR
#define HW_VERSION_MINOR 0
#endif

/*
 * This is the AppImageDescriptor used by the make_can_boot_descriptor.py tool.
 * It must be present in CAN-node style firmwares so the UAVCANv0 bootloader can
 * validate CRC/size/version.
 */
__attribute__((aligned(8)))
boot_app_shared_section app_descriptor_t AppDescriptor = {
	.signature = APP_DESCRIPTOR_SIGNATURE,
	{
		0,
	},
	.image_size = 0,
	.git_hash  = 0,
	.major_version = APP_VERSION_MAJOR,
	.minor_version = APP_VERSION_MINOR,
	.board_id = (uint16_t)((HW_VERSION_MAJOR << 8) | HW_VERSION_MINOR),
	.reserved = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};
#endif

extern "C" {
#include <canard.h>
}

static void print_usage()
{
	PX4_INFO("usage:\n\tuavcannode_canard {start|status|stop}");
}

static int uavcannode_canard_start(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	// Started by the bootloader, we must pet it
	watchdog_pet();

#if defined(GPIO_CAN_TERM)
	int32_t can_term = 0;
	param_get(param_find("CANNODE_TERM"), &can_term);
	px4_arch_gpiowrite(GPIO_CAN_TERM, (can_term != 0));
#endif

	// CAN bitrate
	int32_t bitrate = 0;

	// Node ID
	int32_t node_id = 0;

	// Did the bootloader auto baud and get a node ID allocated?
	int valid = -1;
	bootloader_app_shared_t shared;

	if (board_app_shared_read) {
		valid = board_app_shared_read(&shared, BootLoader);

	} else {
		valid = bootloader_app_shared_read(&shared, BootLoader);
	}

	if (valid == 0) {
		bitrate = static_cast<int32_t>(shared.bus_speed);
		node_id = static_cast<int32_t>(shared.node_id);

		// Invalidate to prevent deja vu
		bootloader_app_shared_invalidate();

	} else {
		// Node ID (0 means use dynamic allocation)
#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
		if (!board_booted_by_px4()) {
			node_id = 0;
			bitrate = 1000000;

		} else
#endif
		{
			(void)param_get(param_find("CANNODE_BITRATE"), &bitrate);
		}
	}

	// Use a static node ID if the parameter is set and in range
	int32_t cannode_node_id = 0;
	param_get(param_find("CANNODE_NODE_ID"), &cannode_node_id);

	static constexpr int32_t NodeIdMax = 127;

	if (cannode_node_id < 0 || cannode_node_id > NodeIdMax) {
		PX4_ERR("Invalid static node ID %ld, using dynamic allocation", cannode_node_id);
		node_id = 0;

	} else {
		node_id = cannode_node_id;
	}

	// Persist the node ID for the bootloader
	bootloader_app_shared_t shared_write {};
	shared_write.node_id = static_cast<uint32_t>(node_id);
	shared_write.bus_speed = 0; // we always want to autobaud
	bootloader_app_shared_write(&shared_write, BootLoader);

	if (
#if defined(SUPPORT_ALT_CAN_BOOTLOADER)
		board_booted_by_px4() &&
#endif
		(node_id < 0 || node_id > NodeIdMax)) {
		PX4_ERR("Invalid Node ID %ld", node_id);
		return 1;
	}

	PX4_INFO("Node ID %ld, bitrate %ld", node_id, bitrate);
	return UavcannodeCanard::start(node_id, bitrate);
}

extern "C" __EXPORT int uavcannode_canard_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		return 1;
	}

	if (!std::strcmp(argv[1], "start")) {
		if (UavcannodeCanard::instance()) {
			PX4_ERR("already started");
			return 1;
		}

		return uavcannode_canard_start(argc, argv);
	}

	UavcannodeCanard *const inst = UavcannodeCanard::instance();

	if (!inst) {
		PX4_ERR("application not running");
		return 1;
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->PrintInfo();
		return 0;
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		return 0;
	}

	print_usage();
	return 1;
}
