
#include "arming_status.hpp"

UavcanArmingStatus::UavcanArmingStatus(uavcan::INode &node) :
	_arming_status_pub(node),
	_timer(node)
{
	_arming_status_pub.setPriority(uavcan::TransferPriority::Default);
}

int UavcanArmingStatus::init()
{
	/*
	 * Setup timer and call back function for periodic updates
	 */
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanArmingStatus::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

void UavcanArmingStatus::periodic_update(const uavcan::TimerEvent &)
{
	actuator_armed_s actuator_armed;

	if (_actuator_armed_sub.update(&actuator_armed)) {
        uavcan::equipment::safety::ArmingStatus msg;

		if ((actuator_armed.armed || actuator_armed.prearmed) && !actuator_armed.lockdown) {
            msg.status = msg.STATUS_FULLY_ARMED;

		} else {
            msg.status = msg.STATUS_DISARMED;
		}

		(void)_arming_status_pub.broadcast(msg);
	}
}
