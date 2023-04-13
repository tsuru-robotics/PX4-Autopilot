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

#ifndef RATE_INT_CONTROL_HPP
#define RATE_INT_CONTROL_HPP

#include <uORB/topics/integral_part_ratecontrol.h>

class MavlinkStreamRateIntControl : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRateIntControl(mavlink); }

	static constexpr const char *get_name_static() { return "RATE_INT_CONTROL"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_RATE_INT_CONTROL; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _int_rate_sub.advertised() ? MAVLINK_MSG_ID_RATE_INT_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamRateIntControl(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _int_rate_sub{ORB_ID(integral_part_ratecontrol)};

	bool send() override
	{
        bool res = false;
        integral_part_ratecontrol_s int_rate_part;

        if (_int_rate_sub.update(&int_rate_part)) {
            mavlink_rate_int_control_t msg{};
            msg.time_usec = int_rate_part.timestamp;
            msg.x = int_rate_part.x;
            msg.y = int_rate_part.y;
            msg.z = int_rate_part.z;

            mavlink_msg_rate_int_control_send_struct(_mavlink->get_channel(), &msg);
            // PX4_INFO("Sending mavlink_rate_int_control_t with x=%f y=%f z=%f!", (double)msg.x, (double)msg.y, (double)msg.z);
            res = true;
        }

		return res;
	}
};

#endif // RATE_INT_CONTROL_HPP
