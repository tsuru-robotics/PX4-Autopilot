/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef KAIKEN_TM_HPP
#define KAIKEN_TM_HPP

#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/health_report.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/failsafe_flags.h>
#include "commander/failsafe/framework.h"

class MavlinkStreamKaikenTm : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamKaikenTm(mavlink); }

	static constexpr const char *get_name_static() { return "KAIKEN_TM"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_FMU_TM; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_FMU_TM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	explicit MavlinkStreamKaikenTm(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps), 0};
	uORB::Subscription _land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _gpos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags)};
	uORB::Subscription _health_report_sub{ORB_ID(health_report)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	using health_component_t = events::px4::enums::health_component_t;

	bool _offboard_activated_once{false};

	void fillOutComponent(const health_report_s &health_report, FMU_COMPONENT_STATUS mav_component,
			      health_component_t health_component, mavlink_fmu_tm_t &msg)
	{
		if (health_report.health_is_present_flags & (uint64_t)health_component) {
			msg.component_present |= mav_component;

			// Check health only if present
			if (((health_report.arming_check_error_flags | health_report.arming_check_warning_flags |
			health_report.health_error_flags | health_report.health_warning_flags) & (uint64_t)health_component) == 0) {
				msg.component_health |= mav_component;
				}
		}
	}

	bool send() override
	{
		mavlink_fmu_tm_t msg{};

		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		vehicle_control_mode_s vehicle_control_mode{};
		_vehicle_control_mode_sub.copy(&vehicle_control_mode);

		health_report_s health_report{};
		_health_report_sub.copy(&health_report);

		// gps
		sensor_gps_s gps;
		if (_sensor_gps_sub.copy(&gps)) {
			// abs time
			uint64_t dt_usec{0};
			uint64_t unix_epoch{0};
			dt_usec = hrt_absolute_time() - gps.timestamp;
			unix_epoch = gps.time_utc_usec + dt_usec;
			PX4_DEBUG("TIME_SYNC unix_epoch=%lld, dt=%lld usec", (long long int)unix_epoch, (long long int)dt_usec);
			// If the time is before 2001-01-01, it's probably the default 2000
			if (unix_epoch > 978307200000000) {
				msg.time = unix_epoch;
				msg.flags |= FMU_TM_FLAGS_TIME_VALID;
			}
			// fix_type
			msg.fix_type = gps.fix_type;
			// satellites
			msg.satellites_visible = gps.satellites_used;
			// rtcm_rate
			msg.rtcm_rate_wifi = abs(gps.rtcm_rate_wifi * 10.0f);
			msg.rtcm_rate_lora = abs(gps.rtcm_rate_lora * 10.0f);
		}

		// flight_state
		vehicle_land_detected_s land_detected{};
		if (_land_detected_sub.copy(&land_detected)) {

			if (vehicle_status.timestamp > 0 && land_detected.timestamp > 0) {

				if (land_detected.landed) {
					msg.flight_state |= UTM_FLIGHT_STATE_GROUND;

				} else {
					msg.flight_state |= UTM_FLIGHT_STATE_AIRBORNE;
				}

			} else {
				msg.flight_state |= UTM_FLIGHT_STATE_UNKNOWN;
			}
		}

		// position
		vehicle_global_position_s gpos;
		vehicle_local_position_s lpos;
		if (_gpos_sub.copy(&gpos) && _lpos_sub.copy(&lpos)) {

			// altitude
			if (lpos.z_valid && lpos.z_global) {
				msg.alt = (-lpos.z + lpos.ref_alt) * 1000.0f;
				msg.flags |= FMU_TM_FLAGS_ALT_VALID;

			} else {
				// fall back to baro altitude
				vehicle_air_data_s air_data{};
				_air_data_sub.copy(&air_data);

				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter * 1000.0f;
					msg.flags |= FMU_TM_FLAGS_ALT_VALID;
				}
			}

			// lat, lon, heading
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;
			msg.hdg = math::degrees(matrix::wrap_2pi(lpos.heading)) * 100.0f;
			msg.flags |= FMU_TM_FLAGS_POS_VALID;
		}

		// estimator status
		estimator_status_flags_s estimator_status_flags;
		if (_estimator_status_flags_sub.copy(&estimator_status_flags)) {
			if (estimator_status_flags.cs_gps_hgt == 1) {
				msg.flags |= FMU_TM_FLAGS_ALT_GPS;
			}
		}

		// prearm check
		if (health_report.can_arm_mode_flags & (1u << vehicle_status.nav_state)) {
			msg.flags |= FMU_TM_FLAGS_PREARM_CHECK;
		}

		// Components
		fillOutComponent(health_report, FMU_COMPONENT_STATUS_GYRO, health_component_t::gyro, msg);
		fillOutComponent(health_report, FMU_COMPONENT_STATUS_ACCEL, health_component_t::accel, msg);
		fillOutComponent(health_report, FMU_COMPONENT_STATUS_MAG, health_component_t::magnetometer, msg);
		fillOutComponent(health_report, FMU_COMPONENT_STATUS_BARO, health_component_t::absolute_pressure, msg);
		fillOutComponent(health_report, FMU_COMPONENT_STATUS_GPS, health_component_t::gps, msg);
		fillOutComponent(health_report, FMU_COMPONENT_STATUS_BATTERY, health_component_t::battery, msg);

		// Battery
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);
		if (battery_status.connected) {
			msg.voltage_battery = battery_status.voltage_filtered_v * 1000.0f;
			msg.current_battery = battery_status.current_filtered_a * 100.0f;
			msg.battery_remaining = ceilf(battery_status.remaining * 100.0f);

		} else {
			msg.voltage_battery = UINT16_MAX;
			msg.current_battery = -1;
			msg.battery_remaining = -1;
		}

		// Check if offboard is active
		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			_offboard_activated_once = true;
		} else if (!vehicle_control_mode.flag_armed) {
			// drop flag when offboard not active and disarmed
			_offboard_activated_once = false;
		}

		// Specify failsafe
		if (vehicle_status.failsafe) {
			// Irreversible failsafe is activated
			if ((FailsafeBase::Action)vehicle_status.failsafe_action_selected > FailsafeBase::Action::Warn) {
				msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_ACTION;
			}

			failsafe_flags_s failsafe_flags{};
			if (_failsafe_flags_sub.copy(&failsafe_flags)) {
				if (failsafe_flags.battery_warning > 0 || failsafe_flags.battery_unhealthy) {
					msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_BATTERY;
				}
				if (failsafe_flags.fd_critical_failure) {
					msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_CRITICAL_ATTITUDE;
				}
				if (_offboard_activated_once && failsafe_flags.offboard_control_signal_lost
					&& (failsafe_flags.mode_req_offboard_signal & (1u << vehicle_status.nav_state))) {
					msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_OFFBOARD_LOSS;
				}
				if (failsafe_flags.global_position_invalid) {
					msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_POSITION_LOSS;
				}

				switch (failsafe_flags.geofence_breached) {
					case 1:
						msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_SOFTFENCE;
						break;
					case 2:
						msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_HARDFENCE;
						break;
					case 3:
						msg.failsafe_flags |= FMU_FAILSAFE_FLAGS_PATH;
						break;
				}
			}
		}

		mavlink_msg_fmu_tm_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

#endif // KAIKEN_TM_HPP
