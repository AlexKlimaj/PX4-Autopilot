#include "UavcannodeCanard.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>

#include <cstring>

#include <px4_platform_common/micro_hal.h>

#include <parameters/param.h>

#include <cmath>

#if defined(CONFIG_UAVCANNODE_FLOW_MEASUREMENT)
# include <uORB/topics/vehicle_optical_flow.h>
#endif

#if defined(CONFIG_UAVCANNODE_RANGE_SENSOR_MEASUREMENT)
# include <uORB/topics/distance_sensor.h>
#endif

#if defined(CONFIG_UAVCANNODE_RAW_IMU)
# include <uORB/topics/vehicle_imu.h>
#endif

#if defined(CONFIG_UAVCANNODE_GNSS_FIX)
# include <uORB/topics/sensor_gps.h>
#endif

#if defined(CONFIG_UAVCANNODE_MAGNETIC_FIELD_STRENGTH)
# include <uORB/topics/vehicle_magnetometer.h>
#endif

#if defined(CONFIG_UAVCANNODE_STATIC_PRESSURE) || defined(CONFIG_UAVCANNODE_STATIC_TEMPERATURE)
# include <uORB/topics/sensor_baro.h>
#endif

#if defined(CONFIG_UAVCANNODE_STATIC_TEMPERATURE)
# include <lib/atmosphere/atmosphere.h>
#endif

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND)
# include <uORB/topics/tune_control.h>
#endif

#if defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)
# include <uORB/topics/led_control.h>
#endif

#if defined(CONFIG_UAVCANNODE_SAFETY_BUTTON)
# include <uORB/topics/button_event.h>
#endif

extern "C" int board_get_mfguid(uint8_t mfgid[PX4_CPU_MFGUID_BYTE_LENGTH]);

#if defined(__PX4_NUTTX)
# include <nuttx/config.h>
# ifdef CONFIG_CAN
#  include <nuttx/can/can.h>
#  include <nuttx/fs/fs.h>
#  include <fcntl.h>
#  include <sys/ioctl.h>
#  include <unistd.h>
#  include <errno.h>
#  include <arch/board/board.h>
# endif
#endif

// UAVCAN v0 DSDL signatures (must match v0 spec)
static constexpr uint64_t UavcanNodeStatusSignature = 0x0f0868d0c1a7c6f1ULL;
static constexpr uint16_t UavcanNodeStatusID = 341;

static constexpr uint64_t UavcanGetNodeInfoSignature = 0xee468a8121c46a9eULL;
static constexpr uint16_t UavcanGetNodeInfoID = 1;

static constexpr uint64_t UavcanDynamicNodeIDAllocationSignature = 0x0b2a812620a11d40ULL;
static constexpr uint16_t UavcanDynamicNodeIDAllocationID = 1; // under uavcan.protocol namespace: 1

// Note: uavcan.protocol.dynamic_node_id.Allocation has data_type_id 1 (service id differs)
// but in libcanard, data_type_id is already namespace-resolved. We treat allocation as message with ID 1.

// Application message types needed on ark_can-flow
static constexpr uint16_t ComHexEquipmentFlowMeasurementID = 20200;
static constexpr uint64_t ComHexEquipmentFlowMeasurementSignature = 0x6a908866bcb49c18ULL;

static constexpr uint16_t UavcanEquipmentRangeSensorMeasurementID = 1050;
static constexpr uint64_t UavcanEquipmentRangeSensorMeasurementSignature = 0x68fffe70fc771952ULL;

static constexpr uint16_t UavcanEquipmentAhrsRawImuID = 1003;
static constexpr uint64_t UavcanEquipmentAhrsRawImuSignature = 0x8280632c40e574b5ULL;

static constexpr uint16_t UavcanEquipmentAhrsMagneticFieldStrength2ID = 1002;
static constexpr uint64_t UavcanEquipmentAhrsMagneticFieldStrength2Signature = 0xb6ac0c442430297eULL;

static constexpr uint16_t UavcanEquipmentAirDataStaticPressureID = 1028;
static constexpr uint64_t UavcanEquipmentAirDataStaticPressureSignature = 0xcdc7c43412bdc89aULL;

static constexpr uint16_t UavcanEquipmentAirDataStaticTemperatureID = 1029;
static constexpr uint64_t UavcanEquipmentAirDataStaticTemperatureSignature = 0x49272a6477d96271ULL;

static constexpr uint16_t UavcanEquipmentGnssAuxiliaryID = 1061;
static constexpr uint64_t UavcanEquipmentGnssAuxiliarySignature = 0x9be8bdc4c3dbbfd2ULL;

static constexpr uint16_t UavcanEquipmentGnssFix2ID = 1063;
static constexpr uint64_t UavcanEquipmentGnssFix2Signature = 0xca41e7000f37435fULL;

static constexpr uint16_t UavcanEquipmentIndicationBeepCommandID = 1080;
static constexpr uint64_t UavcanEquipmentIndicationBeepCommandSignature = 0xbe9ea9fec2b15d52ULL;

static constexpr uint16_t UavcanEquipmentIndicationLightsCommandID = 1081;
static constexpr uint64_t UavcanEquipmentIndicationLightsCommandSignature = 0x2031d93c8bdd1ec4ULL;

static constexpr uint16_t ArdupilotIndicationButtonID = 20001;
static constexpr uint64_t ArdupilotIndicationButtonSignature = 0x0645a46efba7466eULL;

static inline float float16_unpack(uint16_t h)
{
	// IEEE754 float16 -> float32
	const uint32_t sign = (static_cast<uint32_t>(h) & 0x8000U) << 16;
	uint32_t exp = (h >> 10) & 0x1FU;
	uint32_t mant = static_cast<uint32_t>(h) & 0x03FFU;

	uint32_t out = 0;

	if (exp == 0) {
		if (mant == 0) {
			out = sign;

		} else {
			// Subnormal
			exp = 1;

			while ((mant & 0x0400U) == 0U) {
				mant <<= 1;
				exp--;
			}

			mant &= 0x03FFU;
			const uint32_t exp32 = (exp - 1U + 127U - 15U) & 0xFFU;
			out = sign | (exp32 << 23) | (mant << 13);
		}

	} else if (exp == 31) {
		// Inf/NaN
		out = sign | 0x7F800000U | (mant << 13);

	} else {
		const uint32_t exp32 = (exp + 127U - 15U) & 0xFFU;
		out = sign | (exp32 << 23) | (mant << 13);
	}

	union {
		uint32_t u;
		float f;
	} v{out};

	return v.f;
}

static inline uint16_t float16_pack(float value)
{
	// IEEE754 float32 -> float16 (round-to-nearest-even). Saturates to inf.
	union {
		float f;
		uint32_t u;
	} v{};

	v.f = value;
	const uint32_t sign = (v.u >> 16) & 0x8000U;
	const uint32_t mantissa = v.u & 0x007FFFFFU;
	int32_t exp = static_cast<int32_t>((v.u >> 23) & 0xFFU);

	if (exp == 255) {
		// Inf/NaN
		if (mantissa != 0U) {
			return static_cast<uint16_t>(sign | 0x7E00U); // qNaN
		}

		return static_cast<uint16_t>(sign | 0x7C00U);
	}

	// Normalize exponent from bias 127 to bias 15
	exp = exp - 127 + 15;

	if (exp >= 31) {
		// Overflow -> Inf
		return static_cast<uint16_t>(sign | 0x7C00U);
	}

	if (exp <= 0) {
		// Subnormal or underflow to zero
		if (exp < -10) {
			return static_cast<uint16_t>(sign);
		}

		// Make mantissa with implicit leading 1
		uint32_t m = mantissa | 0x00800000U;
		const uint32_t shift = static_cast<uint32_t>(1 - exp);
		m = m >> shift;

		// Round
		const uint32_t round_bit = (m >> 12) & 1U;
		const uint32_t sticky = (m & 0x0FFFU) != 0U;
		uint16_t out = static_cast<uint16_t>(sign | (m >> 13));

		if (round_bit && (sticky || (out & 1U))) {
			out++;
		}

		return out;
	}

	// Normalized
	uint32_t m = mantissa;
	// Round mantissa from 23 bits to 10 bits
	const uint32_t round_bit = (m >> 12) & 1U;
	const uint32_t sticky = (m & 0x0FFFU) != 0U;
	uint16_t out = static_cast<uint16_t>(sign | (static_cast<uint32_t>(exp) << 10) | (m >> 13));

	if (round_bit && (sticky || (out & 1U))) {
		out++;
	}

	return out;
}

UavcannodeCanard *UavcannodeCanard::_instance = nullptr;

class UavcannodeCanard::CanDev
{
public:
	CanDev() = default;
	~CanDev() { close(); }

	bool open(uint32_t bitrate)
	{
#if defined(__PX4_NUTTX)
# ifdef CONFIG_CAN

		// Ensure /dev/can0 exists (same pattern as other CAN-based firmwares)
		if (access("/dev/can0", F_OK) != 0) {
#  if defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN)
			FAR struct can_dev_s *can = stm32_caninitialize(1);

			if (can != nullptr) {
				(void)can_register("/dev/can0", can);
			}

#  endif
		}

		_fd = ::open("/dev/can0", O_RDWR);

		if (_fd < 0) {
			PX4_ERR("open /dev/can0 failed (%d)", errno);
			return false;
		}

		// Set bitrate if driver supports it (optional). Many NuttX CAN drivers ignore this ioctl.
#  ifdef CANIOC_SET_BITRATE
		(void)ioctl(_fd, CANIOC_SET_BITRATE, (unsigned long)bitrate);
#  else
		(void)bitrate;
#  endif

		return true;
# else
		(void)bitrate;
		PX4_ERR("CONFIG_CAN not enabled");
		errno = ENOTSUP;
		return false;
# endif
#else
		(void)bitrate;
		PX4_ERR("unsupported OS");
		return false;
#endif
	}

	void close()
	{
#if defined(__PX4_NUTTX)
# ifdef CONFIG_CAN

		if (_fd >= 0) {
			::close(_fd);
			_fd = -1;
		}

# endif
#endif
	}

	int read_frame(uint32_t &can_id, uint8_t *data, uint8_t &len)
	{
#if defined(__PX4_NUTTX)
# ifdef CONFIG_CAN
		struct can_msg_s msg {};
		const ssize_t nread = ::read(_fd, &msg, sizeof(msg));

		if (nread <= 0) {
			return -errno;
		}

		if (static_cast<size_t>(nread) < sizeof(msg)) {
			return -EIO;
		}

		can_id = msg.cm_hdr.ch_id;
		len = msg.cm_hdr.ch_dlc;

		if (len > 8) {
			len = 8;
		}

		memcpy(data, msg.cm_data, len);
		return 0;
# else
		(void)can_id; (void)data; (void)len;
		return -ENOTSUP;
# endif
#else
		(void)can_id; (void)data; (void)len;
		return -ENOTSUP;
#endif
	}

	int write_frame(uint32_t can_id, const uint8_t *data, uint8_t len)
	{
#if defined(__PX4_NUTTX)
# ifdef CONFIG_CAN
		struct can_msg_s msg {};
		msg.cm_hdr.ch_id = can_id;
		msg.cm_hdr.ch_dlc = len;

		if (len > 8) {
			len = 8;
			msg.cm_hdr.ch_dlc = len;
		}

		memcpy(msg.cm_data, data, len);

		const ssize_t nwritten = ::write(_fd, &msg, sizeof(msg));

		if (nwritten <= 0) {
			return -errno;
		}

		return 0;
# else
		(void)can_id; (void)data; (void)len;
		return -ENOTSUP;
# endif
#else
		(void)can_id; (void)data; (void)len;
		return -ENOTSUP;
#endif
	}

private:
	int _fd{-1};
};

static uint64_t monotonic_us()
{
	return hrt_absolute_time();
}

static uint64_t uptime_sec()
{
	return monotonic_us() / 1000000ULL;
}

int UavcannodeCanard::start(int32_t node_id, int32_t bitrate)
{
	if (_instance != nullptr) {
		PX4_WARN("already running");
		return 0;
	}

	_instance = new UavcannodeCanard(node_id, bitrate);

	if (_instance == nullptr) {
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}

	if (!_instance->init_canard()) {
		delete _instance;
		_instance = nullptr;
		return -EINVAL;
	}

	_instance->ScheduleNow();
	return 0;
}

UavcannodeCanard::UavcannodeCanard(int32_t node_id, int32_t bitrate)
	: ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_configured_node_id = node_id;
	_bitrate = bitrate;

	_can = new CanDev();
}

UavcannodeCanard::~UavcannodeCanard()
{
	_should_exit.store(true);
	ScheduleClear();

	if (_can) {
		delete _can;
		_can = nullptr;
	}

	_instance = nullptr;
}

bool UavcannodeCanard::init_canard()
{
	if (_can == nullptr) {
		return false;
	}

	if (!_can->open(static_cast<uint32_t>(_bitrate))) {
		return false;
	}

	canardInit(&_canard, _arena, sizeof(_arena), &on_transfer_reception, &should_accept_transfer, this);

	if (_configured_node_id > 0 && _configured_node_id <= 127) {
		canardSetLocalNodeID(&_canard, static_cast<uint8_t>(_configured_node_id));

	} else {
		canardSetLocalNodeID(&_canard, CANARD_BROADCAST_NODE_ID);
	}

	// Read unique ID for allocation (optional)
	memset(_unique_id, 0, sizeof(_unique_id));
	const int uid_len = board_get_mfguid(_unique_id);
	_unique_id_valid = (uid_len > 0);

	_initialized.store(true);
	_last_1hz_us = monotonic_us();
	_last_cleanup_us = _last_1hz_us;

#if defined(CONFIG_UAVCANNODE_RAW_IMU)
	int32_t cannode_pub_raw_imu = 0;
	(void)param_get(param_find("CANNODE_PUB_IMU"), &cannode_pub_raw_imu);
	_publish_raw_imu = (cannode_pub_raw_imu == 1);
#endif

	// If unconfigured, start allocation flow
	if (_configured_node_id <= 0 || _configured_node_id > 127) {
		_alloc_unique_id_offset = 0;
		_alloc_followup_pending = false;
		_alloc_next_request_us = monotonic_us();
		_alloc_followup_until_us = _alloc_next_request_us + 1000ULL * 1000ULL; // 1s followup window
	}

	return true;
}

void UavcannodeCanard::Run()
{
	if (_should_exit.load()) {
		return;
	}

	if (!_initialized.load()) {
		ScheduleDelayed(100000);
		return;
	}

	process_rx();
	process_tx_queue();

	const uint64_t now_us = monotonic_us();

	if (now_us - _last_1hz_us >= 1000000ULL) {
		_last_1hz_us = now_us;
		publish_node_status();
	}

	// Application publishers: only emit once we have a node ID
	if (canardGetLocalNodeID(&_canard) != CANARD_BROADCAST_NODE_ID) {

#if defined(CONFIG_UAVCANNODE_FLOW_MEASUREMENT)
		vehicle_optical_flow_s optical_flow {};

		if (_flow_sub.update(&optical_flow)) {
			uint8_t payload[21] {};
			uint32_t offset = 0;

			const float integration_interval = optical_flow.integration_timespan_us * 1e-6f;
			canardEncodeScalar(payload, offset, 32, &integration_interval);
			offset += 32;

			for (int i = 0; i < 2; i++) {
				const float v = optical_flow.delta_angle[i];
				canardEncodeScalar(payload, offset, 32, &v);
				offset += 32;
			}

			for (int i = 0; i < 2; i++) {
				const float v = optical_flow.pixel_flow[i];
				canardEncodeScalar(payload, offset, 32, &v);
				offset += 32;
			}

			const uint8_t quality = optical_flow.quality;
			canardEncodeScalar(payload, offset, 8, &quality);
			offset += 8;

			const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
			(void)canardBroadcast(&_canard,
					      ComHexEquipmentFlowMeasurementSignature,
					      ComHexEquipmentFlowMeasurementID,
					      &_flow_tid,
					      CANARD_TRANSFER_PRIORITY_MEDIUM,
					      payload,
					      payload_len);
		}

#endif

#if defined(CONFIG_UAVCANNODE_RANGE_SENSOR_MEASUREMENT)
		distance_sensor_s dist {};

		if (_distance_sensor_sub.update(&dist)) {
			// Legacy mapping mirrors src/drivers/uavcannode/Publishers/RangeSensorMeasurement.hpp
			uint8_t payload[15] {};
			uint32_t offset = 0;

			// uavcan.Timestamp timestamp: UNKNOWN=0
			const uint64_t ts_usec = 0;
			canardEncodeScalar(payload, offset, 56, &ts_usec);
			offset += 56;

			// sensor_id = instance (legacy uses instance=0)
			const uint8_t sensor_id = 0;
			canardEncodeScalar(payload, offset, 8, &sensor_id);
			offset += 8;

			// uavcan.CoarseOrientation beam_orientation_in_body_frame: zeros + orientation_defined=false
			const int8_t axis = 0;

			for (int i = 0; i < 3; i++) {
				canardEncodeScalar(payload, offset, 5, &axis);
				offset += 5;
			}

			const uint8_t orientation_defined = 0;
			canardEncodeScalar(payload, offset, 1, &orientation_defined);
			offset += 1;

			const uint16_t fov_f16 = float16_pack(dist.h_fov);
			canardEncodeScalar(payload, offset, 16, &fov_f16);
			offset += 16;

			// sensor_type (uint5)
			uint8_t sensor_type = 0; // UNDEFINED

			switch (dist.type) {
			case distance_sensor_s::MAV_DISTANCE_SENSOR_LASER:
				sensor_type = 2; // LIDAR
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND:
				sensor_type = 1; // SONAR
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR:
				sensor_type = 3; // RADAR
				break;

			case distance_sensor_s::MAV_DISTANCE_SENSOR_INFRARED:
			default:
				sensor_type = 0; // UNDEFINED
				break;
			}

			canardEncodeScalar(payload, offset, 5, &sensor_type);
			offset += 5;

			// reading_type (uint3)
			uint8_t reading_type = 0; // UNDEFINED
			static constexpr float tolerance = 1e-6f;

			if (dist.current_distance > dist.max_distance) {
				reading_type = 3; // TOO_FAR

			} else if (dist.current_distance < dist.min_distance - tolerance) {
				reading_type = 2; // TOO_CLOSE

			} else if (dist.signal_quality != 0) {
				reading_type = 1; // VALID_RANGE

			} else {
				reading_type = 0; // UNDEFINED
			}

			canardEncodeScalar(payload, offset, 3, &reading_type);
			offset += 3;

			const uint16_t range_f16 = float16_pack(dist.current_distance);
			canardEncodeScalar(payload, offset, 16, &range_f16);
			offset += 16;

			const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
			(void)canardBroadcast(&_canard,
					      UavcanEquipmentRangeSensorMeasurementSignature,
					      UavcanEquipmentRangeSensorMeasurementID,
					      &_range_tid,
					      CANARD_TRANSFER_PRIORITY_MEDIUM,
					      payload,
					      payload_len);
		}

#endif

#if defined(CONFIG_UAVCANNODE_RAW_IMU)

		if (_publish_raw_imu) {
			vehicle_imu_s vehicle_imu{};

			if (_vehicle_imu_sub.update(&vehicle_imu)) {
				uint8_t payload[64] {};
				uint32_t offset = 0;

				// uavcan.Timestamp timestamp: leave UNKNOWN (no timesync in this minimal canard node)
				const uint64_t ts_usec = 0;
				canardEncodeScalar(payload, offset, 56, &ts_usec);
				offset += 56;

				// integration_interval (legacy assigns microseconds as-is)
				const float integration_interval = static_cast<float>(vehicle_imu.delta_angle_dt);
				canardEncodeScalar(payload, offset, 32, &integration_interval);
				offset += 32;

				// rate_gyro_latest float16[3]
				for (int i = 0; i < 3; i++) {
					const float v = (vehicle_imu.delta_angle[i] / static_cast<float>(vehicle_imu.delta_angle_dt)) * 1000000.0f;
					const uint16_t f16 = float16_pack(v);
					canardEncodeScalar(payload, offset, 16, &f16);
					offset += 16;
				}

				// rate_gyro_integral float32[3]
				for (int i = 0; i < 3; i++) {
					const float v = vehicle_imu.delta_angle[i];
					canardEncodeScalar(payload, offset, 32, &v);
					offset += 32;
				}

				// accelerometer_latest float16[3]
				for (int i = 0; i < 3; i++) {
					const float v = (vehicle_imu.delta_velocity[i] / static_cast<float>(vehicle_imu.delta_velocity_dt)) * 1000000.0f;
					const uint16_t f16 = float16_pack(v);
					canardEncodeScalar(payload, offset, 16, &f16);
					offset += 16;
				}

				// accelerometer_integral float32[3]
				for (int i = 0; i < 3; i++) {
					const float v = vehicle_imu.delta_velocity[i];
					canardEncodeScalar(payload, offset, 32, &v);
					offset += 32;
				}

				// covariance is float16[<=36] and last field => TAO. Legacy leaves it empty.

				const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
				(void)canardBroadcast(&_canard,
						      UavcanEquipmentAhrsRawImuSignature,
						      UavcanEquipmentAhrsRawImuID,
						      &_raw_imu_tid,
						      CANARD_TRANSFER_PRIORITY_LOW,
						      payload,
						      payload_len);
			}
		}

#endif

#if defined(CONFIG_UAVCANNODE_MAGNETIC_FIELD_STRENGTH)
		vehicle_magnetometer_s mag {};

		if (_mag_sub.update(&mag)) {
			uint8_t payload[8] {};
			uint32_t offset = 0;

			const uint8_t sensor_id = 0;
			canardEncodeScalar(payload, offset, 8, &sensor_id);
			offset += 8;

			for (int i = 0; i < 3; i++) {
				const uint16_t f16 = float16_pack(mag.magnetometer_ga[i]);
				canardEncodeScalar(payload, offset, 16, &f16);
				offset += 16;
			}

			// magnetic_field_covariance is float16[<=9] and last field => TAO (omit, empty)

			const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
			(void)canardBroadcast(&_canard,
					      UavcanEquipmentAhrsMagneticFieldStrength2Signature,
					      UavcanEquipmentAhrsMagneticFieldStrength2ID,
					      &_mag_tid,
					      CANARD_TRANSFER_PRIORITY_MEDIUM,
					      payload,
					      payload_len);
		}

#endif

#if defined(CONFIG_UAVCANNODE_STATIC_PRESSURE) || defined(CONFIG_UAVCANNODE_STATIC_TEMPERATURE)
		sensor_baro_s baro {};
		const bool baro_updated = _baro_sub.update(&baro);

# if defined(CONFIG_UAVCANNODE_STATIC_PRESSURE)

		if (baro_updated) {
			uint8_t payload[6] {};
			uint32_t offset = 0;

			const float static_pressure = baro.pressure;
			canardEncodeScalar(payload, offset, 32, &static_pressure);
			offset += 32;

			const uint16_t variance = float16_pack(0.0f);
			canardEncodeScalar(payload, offset, 16, &variance);
			offset += 16;

			const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
			(void)canardBroadcast(&_canard,
					      UavcanEquipmentAirDataStaticPressureSignature,
					      UavcanEquipmentAirDataStaticPressureID,
					      &_static_pressure_tid,
					      CANARD_TRANSFER_PRIORITY_MEDIUM,
					      payload,
					      payload_len);
		}

# endif

# if defined(CONFIG_UAVCANNODE_STATIC_TEMPERATURE)

		if (baro_updated) {
			const uint64_t now = now_us;

			if (now - _last_static_temperature_pub_us >= 1000000ULL) {
				_last_static_temperature_pub_us = now;

				uint8_t payload[4] {};
				uint32_t offset = 0;

				const float kelvin = baro.temperature - atmosphere::kAbsoluteNullCelsius;
				const uint16_t temp_f16 = float16_pack(kelvin);
				canardEncodeScalar(payload, offset, 16, &temp_f16);
				offset += 16;

				const uint16_t variance = float16_pack(0.0f);
				canardEncodeScalar(payload, offset, 16, &variance);
				offset += 16;

				const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
				(void)canardBroadcast(&_canard,
						      UavcanEquipmentAirDataStaticTemperatureSignature,
						      UavcanEquipmentAirDataStaticTemperatureID,
						      &_static_temperature_tid,
						      CANARD_TRANSFER_PRIORITY_LOW,
						      payload,
						      payload_len);
			}
		}

# endif
#endif

#if defined(CONFIG_UAVCANNODE_GNSS_FIX)
		sensor_gps_s gps {};

		if (_gps_sub.update(&gps)) {
			// uavcan.equipment.gnss.Auxiliary
			{
				uint8_t payload[16] {};
				uint32_t offset = 0;

				const uint16_t nan_f16 = float16_pack(NAN);
				const uint16_t hdop_f16 = float16_pack(gps.hdop);
				const uint16_t vdop_f16 = float16_pack(gps.vdop);

				// gdop, pdop
				canardEncodeScalar(payload, offset, 16, &nan_f16); offset += 16;
				canardEncodeScalar(payload, offset, 16, &nan_f16); offset += 16;
				// hdop, vdop
				canardEncodeScalar(payload, offset, 16, &hdop_f16); offset += 16;
				canardEncodeScalar(payload, offset, 16, &vdop_f16); offset += 16;
				// tdop, ndop, edop
				canardEncodeScalar(payload, offset, 16, &nan_f16); offset += 16;
				canardEncodeScalar(payload, offset, 16, &nan_f16); offset += 16;
				canardEncodeScalar(payload, offset, 16, &nan_f16); offset += 16;

				const uint8_t sats_visible = static_cast<uint8_t>(gps.satellites_used & 0x7FU);
				const uint8_t sats_used = static_cast<uint8_t>(gps.satellites_used & 0x3FU);
				canardEncodeScalar(payload, offset, 7, &sats_visible);
				offset += 7;
				canardEncodeScalar(payload, offset, 6, &sats_used);
				offset += 6;

				const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
				(void)canardBroadcast(&_canard,
						      UavcanEquipmentGnssAuxiliarySignature,
						      UavcanEquipmentGnssAuxiliaryID,
						      &_gnss_aux_tid,
						      CANARD_TRANSFER_PRIORITY_MEDIUM,
						      payload,
						      payload_len);
			}

			// uavcan.equipment.gnss.Fix2
			{
				// Size is bounded; we reserve enough for basic fields + covariance (6 entries) and pdop.
				uint8_t payload[80] {};
				uint32_t offset = 0;

				// timestamp (uavcan.Timestamp), gnss_timestamp
				const uint64_t ts_unknown = 0;
				canardEncodeScalar(payload, offset, 56, &ts_unknown);
				offset += 56;
				const uint64_t gnss_ts = gps.time_utc_usec;
				canardEncodeScalar(payload, offset, 56, &gnss_ts);
				offset += 56;

				const uint8_t time_standard_utc = 2;
				canardEncodeScalar(payload, offset, 3, &time_standard_utc);
				offset += 3;

				// void13
				offset += 13;

				const uint8_t num_leap_seconds = 0; // UNKNOWN
				canardEncodeScalar(payload, offset, 8, &num_leap_seconds);
				offset += 8;

				const int64_t longitude_deg_1e8 = static_cast<int64_t>(gps.longitude_deg * 1e8);
				const int64_t latitude_deg_1e8 = static_cast<int64_t>(gps.latitude_deg * 1e8);
				const int32_t height_ellipsoid_mm = static_cast<int32_t>(gps.altitude_ellipsoid_m * 1e3);
				const int32_t height_msl_mm = static_cast<int32_t>(gps.altitude_msl_m * 1e3);

				canardEncodeScalar(payload, offset, 37, &longitude_deg_1e8);
				offset += 37;
				canardEncodeScalar(payload, offset, 37, &latitude_deg_1e8);
				offset += 37;
				canardEncodeScalar(payload, offset, 27, &height_ellipsoid_mm);
				offset += 27;
				canardEncodeScalar(payload, offset, 27, &height_msl_mm);
				offset += 27;

				const float v_n = gps.vel_n_m_s;
				const float v_e = gps.vel_e_m_s;
				const float v_d = gps.vel_d_m_s;
				canardEncodeScalar(payload, offset, 32, &v_n);
				offset += 32;
				canardEncodeScalar(payload, offset, 32, &v_e);
				offset += 32;
				canardEncodeScalar(payload, offset, 32, &v_d);
				offset += 32;

				const uint8_t sats_used = static_cast<uint8_t>(gps.satellites_used & 0x3FU);
				canardEncodeScalar(payload, offset, 6, &sats_used);
				offset += 6;

				// status (uint2) mapped from fix_type
				uint8_t status = 0; // NO_FIX

				if (gps.fix_type >= sensor_gps_s::FIX_TYPE_3D) {
					status = 3;

				} else if (gps.fix_type == sensor_gps_s::FIX_TYPE_2D) {
					status = 2;

				} else if (gps.fix_type == sensor_gps_s::FIX_TYPE_NONE) {
					status = 0;

				} else {
					status = 0;
				}

				canardEncodeScalar(payload, offset, 2, &status);
				offset += 2;

				// mode/sub_mode derived from fix_type as legacy intent
				uint8_t mode = 0; // SINGLE
				uint8_t sub_mode = 0;

				if (gps.fix_type == sensor_gps_s::FIX_TYPE_RTCM_CODE_DIFFERENTIAL) {
					mode = 1; // DGPS

				} else if (gps.fix_type == sensor_gps_s::FIX_TYPE_RTK_FLOAT) {
					mode = 2; // RTK
					sub_mode = 0; // RTK_FLOAT

				} else if (gps.fix_type == sensor_gps_s::FIX_TYPE_RTK_FIXED) {
					mode = 2; // RTK
					sub_mode = 1; // RTK_FIXED
				}

				canardEncodeScalar(payload, offset, 4, &mode);
				offset += 4;
				canardEncodeScalar(payload, offset, 6, &sub_mode);
				offset += 6;

				// covariance float16[<=36] (not last field => explicit length)
				const uint8_t cov_len = 6;
				canardEncodeScalar(payload, offset, 6, &cov_len);
				offset += 6;

				const uint16_t eph_f16 = float16_pack(gps.eph);
				const uint16_t epv_f16 = float16_pack(gps.epv);
				const uint16_t sv_f16 = float16_pack(gps.s_variance_m_s);
				// diag: pos X/Y/Z, vel X/Y/Z
				canardEncodeScalar(payload, offset, 16, &eph_f16); offset += 16;
				canardEncodeScalar(payload, offset, 16, &eph_f16); offset += 16;
				canardEncodeScalar(payload, offset, 16, &epv_f16); offset += 16;
				canardEncodeScalar(payload, offset, 16, &sv_f16);  offset += 16;
				canardEncodeScalar(payload, offset, 16, &sv_f16);  offset += 16;
				canardEncodeScalar(payload, offset, 16, &sv_f16);  offset += 16;

				const float pdop_f = (gps.hdop > gps.vdop) ? gps.hdop : gps.vdop;
				const uint16_t pdop_f16 = float16_pack(pdop_f);
				canardEncodeScalar(payload, offset, 16, &pdop_f16);
				offset += 16;

				// ecef_position_velocity is last field => TAO; we omit it (length=0)

				const uint16_t payload_len = static_cast<uint16_t>((offset + 7U) / 8U);
				(void)canardBroadcast(&_canard,
						      UavcanEquipmentGnssFix2Signature,
						      UavcanEquipmentGnssFix2ID,
						      &_gnss_fix2_tid,
						      CANARD_TRANSFER_PRIORITY_HIGH,
						      payload,
						      payload_len);
			}
		}

#endif

#if defined(CONFIG_UAVCANNODE_SAFETY_BUTTON)
		button_event_s safety_button {};

		if (_safety_button_sub.update(&safety_button)) {
			if (safety_button.triggered) {
				// ardupilot.indication.Button: {button=SAFETY, press_time=10}
				uint8_t payload[2] {};
				payload[0] = 1;  // BUTTON_SAFETY
				payload[1] = 10; // 10 * 0.1s
				(void)canardBroadcast(&_canard,
						      ArdupilotIndicationButtonSignature,
						      ArdupilotIndicationButtonID,
						      &_safety_button_tid,
						      CANARD_TRANSFER_PRIORITY_MEDIUM,
						      payload,
						      sizeof(payload));
			}
		}

#endif
	}

	// allocator periodic requests if node_id is not set
	if (canardGetLocalNodeID(&_canard) == CANARD_BROADCAST_NODE_ID && _unique_id_valid) {
		if (now_us >= _alloc_next_request_us) {
			allocation_send_request(!_alloc_followup_pending, _alloc_unique_id_offset);

			// schedule next request
			if (!_alloc_followup_pending) {
				_alloc_followup_pending = true;
				_alloc_followup_until_us = now_us + 1000ULL * 1000ULL; // 1s followup window
			}

			// request period: 250ms..1s (simple deterministic pacing)
			_alloc_next_request_us = now_us + 500000ULL;

			if (_alloc_followup_pending && now_us > _alloc_followup_until_us) {
				// move to next chunk
				_alloc_followup_pending = false;
				_alloc_unique_id_offset = static_cast<uint8_t>(_alloc_unique_id_offset + 6U);

				if (_alloc_unique_id_offset >= sizeof(_unique_id)) {
					_alloc_unique_id_offset = 0;
				}
			}
		}
	}

	if (now_us - _last_cleanup_us >= 1000000ULL) {
		_last_cleanup_us = now_us;
		canardCleanupStaleTransfers(&_canard, monotonic_us());
	}

	ScheduleDelayed(2000);
}

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND)
void UavcannodeCanard::handle_beep_command(const CanardRxTransfer *transfer)
{
	if (transfer == nullptr) {
		return;
	}

	// uavcan.equipment.indication.BeepCommand
	uint16_t freq_f16 = 0;
	uint16_t dur_f16 = 0;
	(void)canardDecodeScalar(transfer, 0, 16, false, &freq_f16);
	(void)canardDecodeScalar(transfer, 16, 16, false, &dur_f16);

	const float frequency_hz = float16_unpack(freq_f16);
	const float duration_s = float16_unpack(dur_f16);

	if (!PX4_ISFINITE(frequency_hz) || !PX4_ISFINITE(duration_s)) {
		return;
	}

	tune_control_s tune{};
	tune.tune_id = 0;
	tune.frequency = static_cast<uint16_t>(frequency_hz);
	tune.duration = static_cast<uint32_t>(1000000.0f * duration_s);
	tune.volume = 0xff;
	tune.timestamp = hrt_absolute_time();
	_tune_control_pub.publish(tune);
}
#endif

#if defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)
void UavcannodeCanard::handle_lights_command(const CanardRxTransfer *transfer)
{
	if (transfer == nullptr) {
		return;
	}

	// uavcan.equipment.indication.LightsCommand
	// With TAO and fixed-size elements (3 bytes each), length is inferred from payload length.
	const size_t payload_len = transfer->payload_len;
	const size_t n = payload_len / 3U;
	const size_t n_cmd = (n > 20U) ? 20U : n;

	for (size_t i = 0; i < n_cmd; i++) {
		const uint32_t base = static_cast<uint32_t>(i * 24U);
		uint8_t light_id = 0;
		uint8_t red5 = 0;
		uint8_t green6 = 0;
		uint8_t blue5 = 0;

		(void)canardDecodeScalar(transfer, base + 0, 8, false, &light_id);
		(void)canardDecodeScalar(transfer, base + 8, 5, false, &red5);
		(void)canardDecodeScalar(transfer, base + 13, 6, false, &green6);
		(void)canardDecodeScalar(transfer, base + 19, 5, false, &blue5);

		if (light_id != _self_light_index) {
			continue;
		}

		const uint32_t red = (static_cast<uint32_t>(red5) * 255U + 15U) / 31U;
		const uint32_t green = (static_cast<uint32_t>(green6) * 255U + 31U) / 63U;
		const uint32_t blue = (static_cast<uint32_t>(blue5) * 255U + 15U) / 31U;

		led_control_s led{};
		led.num_blinks = 0;
		led.priority = led_control_s::MAX_PRIORITY;
		led.mode = led_control_s::MODE_OFF;
		led.led_mask = 0xff;
		led.color = led_control_s::COLOR_OFF;

		if (red != 0 && blue == 0 && green == 0) {
			led.color = led_control_s::COLOR_RED;

		} else if (red == 0 && blue != 0 && green == 0) {
			led.color = led_control_s::COLOR_BLUE;

		} else if (red == 0 && blue == 0 && green != 0) {
			led.color = led_control_s::COLOR_GREEN;

		} else if (red != 0 && blue == 0 && green != 0) {
			led.color = led_control_s::COLOR_YELLOW;

		} else if (red != 0 && blue != 0 && green == 0) {
			led.color = led_control_s::COLOR_PURPLE;

		} else if (red != 0 && blue == 0 && green != 0 && red > green) {
			led.color = led_control_s::COLOR_AMBER;

		} else if (red == 0 && blue != 0 && green != 0) {
			led.color = led_control_s::COLOR_CYAN;

		} else if (red != 0 && blue != 0 && green != 0) {
			led.color = led_control_s::COLOR_WHITE;
		}

		if (led.color != led_control_s::COLOR_OFF) {
			led.mode = led_control_s::MODE_ON;
		}

		led.timestamp = hrt_absolute_time();
		_led_control_pub.publish(led);
	}
}
#endif

void UavcannodeCanard::process_rx()
{
	uint32_t can_id = 0;
	uint8_t data[8] {};
	uint8_t len = 0;

	for (int i = 0; i < 32; i++) {
		const int ret = _can->read_frame(can_id, data, len);

		if (ret != 0) {
			break;
		}

		CanardCANFrame frame {};
		frame.id = can_id;
		frame.data_len = len;
		memcpy(frame.data, data, len);

		(void)canardHandleRxFrame(&_canard, &frame, monotonic_us());
	}
}

void UavcannodeCanard::process_tx_queue()
{
	for (int i = 0; i < 32; i++) {
		CanardCANFrame *frame = canardPeekTxQueue(&_canard);

		if (frame == nullptr) {
			break;
		}

		const int ret = _can->write_frame(frame->id, frame->data, frame->data_len);

		if (ret != 0) {
			// Drop on persistent errors to prevent stalling.
			canardPopTxQueue(&_canard);
			break;
		}

		canardPopTxQueue(&_canard);
	}
}

void UavcannodeCanard::publish_node_status()
{
	// uavcan.protocol.NodeStatus (ID 341)
	// Fields (packed):
	//  uint32 uptime_sec
	//  uint2 health
	//  uint3 mode
	//  uint3 sub_mode
	//  uint16 vendor_specific_status_code
	// We emit simplest: health=0 OK, mode=0 OPERATIONAL, sub_mode=0, vendor=0
	uint8_t payload[7] {};

	const uint32_t up = static_cast<uint32_t>(uptime_sec());
	payload[0] = static_cast<uint8_t>(up);
	payload[1] = static_cast<uint8_t>(up >> 8);
	payload[2] = static_cast<uint8_t>(up >> 16);
	payload[3] = static_cast<uint8_t>(up >> 24);

	// health/mode/sub_mode packed into one byte
	// bits 0-1 health, 2-4 mode, 5-7 sub_mode
	payload[4] = 0;

	payload[5] = 0;
	payload[6] = 0;

	(void)canardBroadcast(&_canard, UavcanNodeStatusSignature, UavcanNodeStatusID, &_node_status_tid,
			      CANARD_TRANSFER_PRIORITY_LOW, payload, sizeof(payload));
}

void UavcannodeCanard::handle_get_node_info(CanardInstance *ins, CanardRxTransfer *transfer)
{
	// Respond with minimal GetNodeInfoResponse.
	// DSDL layout is complex; we keep response minimal but valid:
	// - NodeStatus (7 bytes)
	// - software_version, hardware_version, name, unique_id etc.
	// We follow same minimalist approach as uavcan_canard.

	uint8_t response[80] {};
	size_t offset = 0;

	// NodeStatus (7 bytes)
	const uint32_t up = static_cast<uint32_t>(uptime_sec());
	response[offset + 0] = static_cast<uint8_t>(up);
	response[offset + 1] = static_cast<uint8_t>(up >> 8);
	response[offset + 2] = static_cast<uint8_t>(up >> 16);
	response[offset + 3] = static_cast<uint8_t>(up >> 24);
	response[offset + 4] = 0; // health/mode/sub
	response[offset + 5] = 0;
	response[offset + 6] = 0;
	offset += 7;

	// software_version (major, minor, optional) - uavcan.protocol.SoftwareVersion
	response[offset + 0] = 1; // major
	response[offset + 1] = 0; // minor
	response[offset + 2] = 0; // vcs_commit (optional tag bits) - keep 0
	offset += 3;

	// hardware_version (major, minor)
	response[offset + 0] = 1;
	response[offset + 1] = 0;
	offset += 2;

	// unique_id[16]
	if (_unique_id_valid) {
		memcpy(&response[offset], _unique_id, 16);
	}

	offset += 16;

	// certificate_of_authenticity length (0)
	response[offset++] = 0;

	// name length + name bytes
	static constexpr char name[] = "org.px4.uavcannode";
	const uint8_t name_len = (sizeof(name) - 1U) > 80U ? 0U : static_cast<uint8_t>(sizeof(name) - 1U);
	response[offset++] = name_len;
	memcpy(&response[offset], name, name_len);
	offset += name_len;

	(void)canardRequestOrRespond(ins,
				     transfer->source_node_id,
				     UavcanGetNodeInfoSignature,
				     UavcanGetNodeInfoID,
				     &transfer->transfer_id,
				     transfer->priority,
				     CanardResponse,
				     response,
				     offset);
}

void UavcannodeCanard::handle_allocation_message(CanardRxTransfer *transfer)
{
	// We implement minimal participation: accept a response that assigns our node_id.
	// Allocation message: first field is node_id (7-bit) with flags etc.
	// The simplest robust approach: use libcanard helper canardDecodeScalar.

	if (!_unique_id_valid) {
		return;
	}

	// First 7 bits: node_id, next bit: first_part_of_unique_id
	uint8_t node_id = 0;
	(void)canardDecodeScalar(transfer, 0, 7, false, &node_id);

	uint8_t first_part = 0;
	(void)canardDecodeScalar(transfer, 7, 1, false, &first_part);

	// unique_id len in bytes: 6 bytes chunks are typical; ignore details and just check node_id is valid
	if (node_id > 0 && node_id <= 127 && first_part) {
		canardSetLocalNodeID(&_canard, node_id);
		PX4_INFO("allocated node id %u", node_id);
	}
}

void UavcannodeCanard::allocation_send_request(bool first_part, uint8_t unique_id_offset)
{
	if (!_unique_id_valid) {
		return;
	}

	// Encode Allocation request: node_id=0, first_part flag, unique_id bytes (up to 6)
	uint8_t payload[1 + 6] {};
	const uint8_t chunk_len = 6;

	// node_id=0 (7 bits), first_part flag in MSB
	payload[0] = static_cast<uint8_t>((first_part ? 0x80 : 0x00) | 0x00);

	const size_t copy_offset = (unique_id_offset < sizeof(_unique_id)) ? static_cast<size_t>(unique_id_offset) : 0U;
	const size_t max_copy = (copy_offset + chunk_len <= sizeof(_unique_id)) ? static_cast<size_t>(chunk_len) : (sizeof(
					_unique_id) - copy_offset);
	memcpy(&payload[1], &_unique_id[copy_offset], max_copy);

	(void)canardBroadcast(&_canard,
			      UavcanDynamicNodeIDAllocationSignature,
			      UavcanDynamicNodeIDAllocationID,
			      &_alloc_tid,
			      CANARD_TRANSFER_PRIORITY_LOW,
			      payload,
			      static_cast<uint16_t>(1U + max_copy));
}

void UavcannodeCanard::on_transfer_reception(CanardInstance *ins, CanardRxTransfer *transfer)
{
	UavcannodeCanard *self = reinterpret_cast<UavcannodeCanard *>(canardGetUserReference(ins));

	if (self == nullptr) {
		return;
	}

	if (transfer->transfer_type == CanardTransferTypeRequest && transfer->data_type_id == UavcanGetNodeInfoID) {
		self->handle_get_node_info(ins, transfer);
		return;
	}

	if (transfer->transfer_type == CanardTransferTypeBroadcast && transfer->data_type_id == UavcanDynamicNodeIDAllocationID) {
		self->handle_allocation_message(transfer);
		return;
	}

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND)

	if (transfer->transfer_type == CanardTransferTypeBroadcast && transfer->data_type_id == UavcanEquipmentIndicationBeepCommandID) {
		self->handle_beep_command(transfer);
		return;
	}

#endif

#if defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)

	if (transfer->transfer_type == CanardTransferTypeBroadcast && transfer->data_type_id == UavcanEquipmentIndicationLightsCommandID) {
		self->handle_lights_command(transfer);
		return;
	}

#endif
}

bool UavcannodeCanard::should_accept_transfer(const CanardInstance *ins,
		uint64_t *out_data_type_signature,
		uint16_t data_type_id,
		CanardTransferType transfer_type,
		uint8_t source_node_id)
{
	(void)source_node_id;

	if (transfer_type == CanardTransferTypeRequest && data_type_id == UavcanGetNodeInfoID) {
		*out_data_type_signature = UavcanGetNodeInfoSignature;
		return true;
	}

	if (transfer_type == CanardTransferTypeBroadcast && data_type_id == UavcanDynamicNodeIDAllocationID) {
		*out_data_type_signature = UavcanDynamicNodeIDAllocationSignature;
		return true;
	}

#if defined(CONFIG_UAVCANNODE_BEEP_COMMAND)

	if (transfer_type == CanardTransferTypeBroadcast && data_type_id == UavcanEquipmentIndicationBeepCommandID) {
		*out_data_type_signature = UavcanEquipmentIndicationBeepCommandSignature;
		return true;
	}

#endif

#if defined(CONFIG_UAVCANNODE_LIGHTS_COMMAND)

	if (transfer_type == CanardTransferTypeBroadcast && data_type_id == UavcanEquipmentIndicationLightsCommandID) {
		*out_data_type_signature = UavcanEquipmentIndicationLightsCommandSignature;
		return true;
	}

#endif

	return false;
}

void UavcannodeCanard::PrintInfo()
{
	PX4_INFO("uavcannode_canard: node_id=%u bitrate=%u", (unsigned)canardGetLocalNodeID(&_canard), (unsigned)_bitrate);
}
