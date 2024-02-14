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
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/estimator_status_flags.h>

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

	bool send() override
	{
		mavlink_fmu_tm_t msg{};

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
			// rtcm_injection_rate
			msg.rtcm_rate_ch0 = abs(gps.rtcm_injection_rate);
			msg.rtcm_rate_ch1 = abs(gps.rtcm_injection_rate);
		}

		// flight_state
		vehicle_land_detected_s land_detected{};
		vehicle_status_s vehicle_status{};
		if (_land_detected_sub.copy(&land_detected) && _vehicle_status_sub.copy(&vehicle_status)) {

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

		mavlink_msg_fmu_tm_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

#endif // KAIKEN_TM_HPP
