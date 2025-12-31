/**
 * UAVCAN mode
 *
 *  0 - UAVCAN disabled.
 *  1 - Enables support for UAVCAN sensors without dynamic node ID allocation and firmware update.
 *  2 - Enables support for UAVCAN sensors with dynamic node ID allocation and firmware update.
 *  3 - Enables support for UAVCAN sensors and actuators with dynamic node ID allocation and firmware update. Also sets the motor control outputs to UAVCAN.
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Sensors Manual Config
 * @value 2 Sensors Automatic Config
 * @value 3 Sensors and Actuators (ESCs) Automatic Config
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_ENABLE, 0);

/**
 * UAVCAN Node ID.
 *
 * @min 1
 * @max 125
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_NODE_ID, 1);

/**
 * UAVCAN CAN bus bitrate.
 *
 * @unit bit/s
 * @min 20000
 * @max 1000000
 * @reboot_required true
 * @group UAVCAN
 */
PARAM_DEFINE_INT32(UAVCAN_BITRATE, 1000000);
