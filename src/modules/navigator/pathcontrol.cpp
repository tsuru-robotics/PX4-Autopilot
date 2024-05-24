/****************************************************************************
 *
 *   Copyright (c) 2013,2017 PX4 Development Team. All rights reserved.
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
 * @file pathcontrol.cpp
 * Provides functions for handling the pathcontrol
 *
 * @author Vladimir Savelyev <vmsavelyev@gmail.com>

 */
#include "pathcontrol.h"
#include "navigator.h"
#include <ctype.h>
#include <drivers/drv_hrt.h>


PathControl::PathControl(Navigator *navigator) :
	ModuleParams(navigator),
	_navigator(navigator)
{
}

void PathControl::pathControlUpdate()
{
	if (_navigator->get_vstatus()->nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		path_control_result_s res{};
		vehicle_local_position_setpoint_s setpoint{};
		_vehicle_local_position_setpoint_sub.copy(&setpoint);

		float dx = setpoint.x - _navigator->get_local_position()->x;
		float dy = setpoint.y - _navigator->get_local_position()->y;
		res.deviation = sqrtf(dx*dx + dy*dy);

		res.inside_acc_r = (res.deviation < _param_pc_acc_radius.get()) ? true :false;
		res.timestamp = hrt_absolute_time();

		if (res.inside_acc_r) {
			_last_time_inside_path_acc_r_us = hrt_absolute_time();
			res.breached = false;
		} else {
			if (res.timestamp  - _last_time_inside_path_acc_r_us < (uint32_t)1e6) {
				res.breached = true;
			} else {
				res.breached = false;
			}
		}

		res.action = _param_pc_action.get();
		_path_control_result_pub.publish(res);
	}

}

