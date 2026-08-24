# DroneCAN ESCs

PX4 supports DroneCAN compliant ESCs.
For more information, see the following articles for specific hardware/firmware:

- [PX4 Sapog ESC Firmware](sapog.md)
  - [Holybro Kotleta 20](holybro_kotleta.md)
- [Zubax Telega](zubax_telega.md)
- [Vertiq](../peripherals/vertiq.md) (larger modules)
- [VESC Project](../peripherals/vesc.md)
- [RaccoonLab Cyphal and DroneCAN PWM nodes](raccoonlab_nodes.md)

## Hardware Configuration

General DroneCAN hardware configuration is covered in [DroneCAN > Hardware Setup](../dronecan/index.md#hardware-setup).

DroneCAN ESCs should be on their own dedicated CAN interface(s) because ESC messages can saturate the bus and starve other nodes of bandwidth.

## PX4 Configuration

DroneCAN peripherals are configured by following the procedure outlined in [DroneCAN](../dronecan/index.md).

In addition to the general setup, such as setting `UAVCAN_ENABLE` to `3`:

- Select the specific CAN interface(s) used for ESC data output using the [UAVCAN_ESC_IFACE](../advanced_config/parameter_reference.md#UAVCAN_ESC_IFACE) parameter (all that all interfaces are selected by default).
- Configure the [motor order and servo outputs](../config/actuators.md).

When ESC output is enabled, PX4 publishes [ArmingStatus](https://dronecan.github.io/Specification/7._List_of_standard_data_types/#armingstatus) (`STATUS_FULLY_ARMED` while armed or during QGC actuator tests).
ESCs that require that message work for both flight and the Actuators page without setting [UAVCAN_PUB_ARM](../advanced_config/parameter_reference.md#UAVCAN_PUB_ARM).
