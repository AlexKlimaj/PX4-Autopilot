/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file optical_flow_control.cpp
 * Control functions for optical flow fusion
 */

#include "ekf.h"

void Ekf::controlOpticalFlowFusion(const imuSample &imu_delayed)
{
	const bool flow_was_ready = _flow_data_ready;

	if (_flow_buffer) {
		// We don't fuse flow data immediately because we have to wait for the mid integration point to fall behind the fusion time horizon.
		// This means we stop looking for new data until the old data has been fused, unless we are not fusing optical flow,
		// in this case we need to empty the buffer
		const bool outdated = (_time_delayed_us > (_flow_sample_delayed.time_us + uint32_t(1e6f * _flow_sample_delayed.dt)));

		if (!_flow_data_ready || outdated) {
			_flow_data_ready = _flow_buffer->pop_first_older_than(imu_delayed.time_us, &_flow_sample_delayed);
			printf("delayed = %f, flow = %f, dt = %f\n", static_cast<double>(_time_delayed_us)/1e6, static_cast<double>(_flow_sample_delayed.time_us)/1e6, (double)_flow_sample_delayed.dt);

			// Reset the accumulators for each new sample
			_imu_del_ang_of.zero();
			_delta_time_of = 0.f;
		}
	}

	// Accumulate autopilot gyro data across the same time interval as the flow sensor
	const Vector3f delta_angle(imu_delayed.delta_ang - (getGyroBias() * imu_delayed.delta_ang_dt));

	if (flow_was_ready && _flow_data_ready) {
		_imu_del_ang_of += delta_angle;
		_delta_time_of += imu_delayed.delta_ang_dt;
	}

	// Wait until the midpoint of the flow sample has fallen behind the fusion time horizon
	bool flow_delayed = false;

	if (_flow_data_ready) {
		flow_delayed = (_time_delayed_us >= (_flow_sample_delayed.time_us + uint32_t(1e6f * _flow_sample_delayed.dt) / 2));
		// printf("delayed = %f, flow = %f, dt = %f\n", static_cast<double>(_time_delayed_us)/1e6, static_cast<double>(_flow_sample_delayed.time_us)/1e6, (double)_flow_sample_delayed.dt);
		printf("pass = %s\n", flow_delayed?"y":"n");
	}

	// New optical flow data is available and is ready to be fused when the midpoint of the sample falls behind the fusion time horizon
	if (flow_delayed) {
		if ((_flow_sample_delayed.quality == 0) && (_flow_sample_delayed.dt < FLT_EPSILON)) {
			// handle special case of SITL and PX4Flow where dt is forced to
			// zero when the quaity is 0
			_flow_sample_delayed.dt = 0.02f;
		}

		const int32_t min_quality = _control_status.flags.in_air
					    ? _params.flow_qual_min
					    : _params.flow_qual_min_gnd;

		const bool is_quality_good = (_flow_sample_delayed.quality >= min_quality);
		const bool is_magnitude_good = !_flow_sample_delayed.flow_xy_rad.longerThan(_flow_sample_delayed.dt * _flow_max_rate);
		const bool is_tilt_good = (_R_to_earth(2, 2) > _params.range_cos_max_tilt);
		const bool is_dt_valid = _flow_sample_delayed.dt > FLT_EPSILON;

		if (is_quality_good
		    && is_magnitude_good
		    && is_tilt_good
		    && is_dt_valid) {
			// compensate for body motion to give a LOS rate
			calcOptFlowBodyRateComp();
			_flow_compensated_XY_rad = _flow_sample_delayed.flow_xy_rad - _flow_sample_delayed.gyro_rate_integral.xy();

		} else {
			// don't use this flow data and wait for the next data to arrive
			_flow_data_ready = false;
			flow_delayed = false;
			printf("dt = %f, q = %d\n", (double)_flow_sample_delayed.dt, _flow_sample_delayed.quality);
			_flow_compensated_XY_rad.setZero();
		}
	}

	if (flow_delayed) {
		updateOptFlow(_aid_src_optical_flow);

		// Check if we are in-air and require optical flow to control position drift
		bool is_flow_required = _control_status.flags.in_air
					      && (_control_status.flags.inertial_dead_reckoning // is doing inertial dead-reckoning so must constrain drift urgently
						  || isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.opt_flow));

		// inhibit use of optical flow if motion is unsuitable and we are not reliant on it for flight navigation
		const bool inhibit_flow_use = (!isTerrainEstimateValid() && !is_flow_required)
					      || !_control_status.flags.tilt_align;

		// Handle cases where we are using optical flow but we should not use it anymore
		if (_control_status.flags.opt_flow) {
			if (!(_params.flow_ctrl == 1)
			    || inhibit_flow_use) {

				stopFlowFusion();
				return;
			}
		}

		// use a relaxed time criteria to enable it to coast through bad range finder data
		const bool terrain_available = isTerrainEstimateValid() || isRecent(_aid_src_terrain_range_finder.time_last_fuse, (uint64_t)10e6);

		// optical flow fusion mode selection logic
		if ((_params.flow_ctrl == 1) // optical flow has been selected by the user
		    && !_control_status.flags.opt_flow // we are not yet using flow data
		    && flow_delayed
		    && !inhibit_flow_use
		    && !isRecent(_aid_src_optical_flow.time_last_fuse, (uint64_t)2e6)
		    && terrain_available
		   ) {
			// set the flag and reset the fusion timeout
			ECL_INFO("starting optical flow fusion");

			_innov_check_fail_status.flags.reject_optflow_X = false;
			_innov_check_fail_status.flags.reject_optflow_Y = false;

			// if we are not using GPS or external vision aiding, then the velocity and position states and covariances need to be set
			if (!isHorizontalAidingActive()) {
				ECL_INFO("reset velocity to flow");
				_information_events.flags.reset_vel_to_flow = true;
				resetHorizontalVelocityTo(_flow_vel_ne, calcOptFlowMeasVar(_flow_sample_delayed));

				// reset position, estimate is relative to initial position in this mode, so we start with zero error
				if (!_control_status.flags.in_air) {
					ECL_INFO("reset position to zero");
					resetHorizontalPositionTo(Vector2f(0.f, 0.f), 0.f);
					_last_known_pos.xy() = _state.pos.xy();

				} else {
					_information_events.flags.reset_pos_to_last_known = true;
					ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));
					resetHorizontalPositionTo(_last_known_pos.xy(), 0.f);
				}

				_aid_src_optical_flow.test_ratio[0] = 0.f;
				_aid_src_optical_flow.test_ratio[1] = 0.f;
				_aid_src_optical_flow.innovation_rejected = false;
				_aid_src_optical_flow.time_last_fuse = _time_delayed_us;

				_control_status.flags.opt_flow = true;
				return;
			}

			// otherwise enable opt_flow and continue to fusion
			_control_status.flags.opt_flow = true;
		}

		if (_control_status.flags.opt_flow) {
			if (flow_delayed) {
				if (terrain_available) {
					// Fuse optical flow LOS rate observations into the main filter only if height above ground has been updated recently
					fuseOptFlow();
					_last_known_pos.xy() = _state.pos.xy();

					// handle the case when we have optical flow, are reliant on it, but have not been using it for an extended period
					if (isTimedOut(_aid_src_optical_flow.time_last_fuse, _params.no_aid_timeout_max)
					&& !isOtherSourceOfHorizontalAidingThan(_control_status.flags.opt_flow)
					) {
						ECL_INFO("reset velocity to flow");
						_information_events.flags.reset_vel_to_flow = true;
						resetHorizontalVelocityTo(_flow_vel_ne, calcOptFlowMeasVar(_flow_sample_delayed));

						// reset position, estimate is relative to initial position in this mode, so we start with zero error
						ECL_INFO("reset position to last known (%.3f, %.3f)", (double)_last_known_pos(0), (double)_last_known_pos(1));
						_information_events.flags.reset_pos_to_last_known = true;
						resetHorizontalPositionTo(_last_known_pos.xy(), 0.f);

						_aid_src_optical_flow.test_ratio[0] = 0.f;
						_aid_src_optical_flow.test_ratio[1] = 0.f;
						_aid_src_optical_flow.innovation_rejected = false;
						_aid_src_optical_flow.time_last_fuse = _time_delayed_us;
					}
				}

				_flow_data_ready = false;
				flow_delayed = false;
			}

			if (!terrain_available || !isRecent(_aid_src_optical_flow.time_last_fuse, (uint64_t)5e6)) {
				stopFlowFusion();
			}
		}

	} else if (_control_status.flags.opt_flow && !isRecent(_flow_sample_delayed.time_us, (uint64_t)10e6)) {

		stopFlowFusion();
	}
}

void Ekf::stopFlowFusion()
{
	if (_control_status.flags.opt_flow) {
		ECL_INFO("stopping optical flow fusion");
		_control_status.flags.opt_flow = false;

		resetEstimatorAidStatus(_aid_src_optical_flow);
	}
}

void Ekf::calcOptFlowBodyRateComp()
{
	_ref_body_rate.zero();
	_measured_body_rate.zero();

	if ((_delta_time_of > FLT_EPSILON)
	    && (_flow_sample_delayed.dt > FLT_EPSILON)) {
		_ref_body_rate = -_imu_del_ang_of / _delta_time_of; // flow gyro has opposite sign convention
		_measured_body_rate = _flow_sample_delayed.gyro_rate_integral / _flow_sample_delayed.dt;

		// calculate the bias estimate using  a combined LPF and spike filter
		_flow_gyro_bias = _flow_gyro_bias * 0.99f + matrix::constrain(_measured_body_rate - _ref_body_rate, -0.1f, 0.1f) * 0.01f;
	}

	if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate_integral(0)) || !PX4_ISFINITE(_flow_sample_delayed.gyro_rate_integral(1))) {
		_flow_sample_delayed.gyro_rate_integral = _ref_body_rate * _flow_sample_delayed.dt;

	} else if (!PX4_ISFINITE(_flow_sample_delayed.gyro_rate_integral(2))) {
		// Some flow modules only provide X ind Y angular rates. If this is the case, complete the vector with our own Z gyro
		_flow_sample_delayed.gyro_rate_integral(2) = _ref_body_rate(2) * _flow_sample_delayed.dt;
	}
	_ref_body_rate.print();
	_measured_body_rate.print();
}
