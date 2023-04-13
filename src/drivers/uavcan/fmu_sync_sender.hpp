#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/kaiken/SyncVel.hpp>
#include <uavcan/kaiken/SyncRate.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/integral_part_velocitycontrol.h>
#include <uORB/topics/integral_part_ratecontrol.h>

class UavcanFmuSyncSender
{
public:
    UavcanFmuSyncSender(uavcan::INode &node);
    ~UavcanFmuSyncSender() = default;

	/*
	* setup periodic updater
	*/
	int init();

private:
	/*
	 * Max update rate to avoid excessive bus traffic
	 */
	static constexpr unsigned MAX_RATE_HZ = 2;

	/*
	 * Setup timer and call back function for periodic updates
	 */
	void periodic_update(const uavcan::TimerEvent &);

	typedef uavcan::MethodBinder<UavcanFmuSyncSender *, void (UavcanFmuSyncSender::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	/*
	 * Subscribe
	 */
	uORB::Subscription _velocity_sub{ORB_ID(integral_part_velocitycontrol)};
    uORB::Subscription _rate_sub{ORB_ID(integral_part_ratecontrol)};

    /*
     * Publish
     */
    uavcan::Publisher<uavcan::kaiken::SyncVel>  _velocity_pub;
    uavcan::Publisher<uavcan::kaiken::SyncRate> _rate_pub;
	uavcan::TimerEventForwarder<TimerCbBinder> _timer;
};
