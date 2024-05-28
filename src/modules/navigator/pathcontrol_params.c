/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file pathcontrol_params.c
 *
 * Parameters for pathcontrol
 *
 * @author Vladimir Savelyev <vmsavelyev@gmail.com>
 */

/*
 * Pathcontrol parameters, accessible via MAVLink
 */


/**
 * Acceptance radius for path control in offboard mode.
 *
 * Maximum distance in meters the vehicle can be from its setpoint.
 *
 * @unit m
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Patchcontrol
 */
PARAM_DEFINE_FLOAT(PC_ACCEPT_R, 3.5);

/**
 * Acceptance radius violation action.
 *
 * Note: Setting this value to 4 enables flight termination,
 * which will kill the vehicle on violation of the fence.
 *
 * @min 0
 * @max 5
 * @value 0 None
 * @value 1 Warning
 * @value 2 Hold mode
 * @value 3 Return mode
 * @value 4 Terminate
 * @value 5 Land mode
 * @group Patchcontrol
 */
PARAM_DEFINE_INT32(PC_ACTION, 1);
