#include "fmu_sync_sender.hpp"

UavcanFmuSyncSender::UavcanFmuSyncSender(uavcan::INode &node) :
        _velocity_pub(node),
        _rate_pub(node),
        _timer(node)
{
    _velocity_pub.setPriority(uavcan::TransferPriority::MiddleLower);
    _rate_pub.setPriority(uavcan::TransferPriority::MiddleLower);
}

int UavcanFmuSyncSender::init()
{
	/*
	 * Setup timer and call back function for periodic updates
	 */
	if (!_timer.isRunning()) {
        _timer.setCallback(TimerCbBinder(this, &UavcanFmuSyncSender::periodic_update));
        _timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

void UavcanFmuSyncSender::periodic_update(const uavcan::TimerEvent &)
{
	if (_velocity_sub.updated()) {
        integral_part_velocitycontrol_s velocity_uorb_topic{};
        _velocity_sub.copy(&velocity_uorb_topic);
        uavcan::kaiken::SyncVel velocity_uavcan{};
        velocity_uavcan.velocity[0] = velocity_uorb_topic.x;
        velocity_uavcan.velocity[1] = velocity_uorb_topic.y;
        velocity_uavcan.velocity[2] = velocity_uorb_topic.z;
        _velocity_pub.broadcast(velocity_uavcan);
	}

    if (_rate_sub.updated()) {
        integral_part_ratecontrol_s rate_uorb_topic{};
        _rate_sub.copy(&rate_uorb_topic);
        uavcan::kaiken::SyncRate rate_uavcan{};
        rate_uavcan.rate[0] = rate_uorb_topic.x;
        rate_uavcan.rate[1] = rate_uorb_topic.y;
        rate_uavcan.rate[2] = rate_uorb_topic.z;
        _rate_pub.broadcast(rate_uavcan);
    }
}