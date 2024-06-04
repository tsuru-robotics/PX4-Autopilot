/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file pathcontrol
 * Provides functions for handling the pathcontrol
 *
 * @author Vladimir Savelyev <vmsavelyev@gmail.com>
 */

#pragma once

#include <float.h>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/path_control_result.h>

class Navigator;

class PathControl : public ModuleParams
{
public:
	PathControl(Navigator *navigator);

	~PathControl() = default;

	void pathControlUpdate();

private:

	Navigator   *_navigator{nullptr};

	uORB::Subscription _vehicle_local_position_setpoint_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Publication<path_control_result_s> _path_control_result_pub{ORB_ID(path_control_result)};

	hrt_abstime _last_time_inside_path_acc_r_us{0};	///< last system time in usec when drone was within path accepnatce radius

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PC_ACCEPT_R>) _param_pc_acc_radius,
		(ParamInt<px4::params::PC_ACTION>)     _param_pc_action
	)
};
