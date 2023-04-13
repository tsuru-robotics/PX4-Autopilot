#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/kaiken/SyncVel.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/integral_part_velocitycontrol.h>

class UavcanFmuSyncReceiver
{
public:
    UavcanFmuSyncReceiver(uavcan::INode &node) : _velocity_sub(node), _rate_sub(node) {}
    ~UavcanFmuSyncReceiver() = default;

    int init()
    {
        int res = _velocity_sub.start(VelMessageCbBinder(this, &UavcanFmuSyncReceiver::velocity_sub_cb));
        if (res < 0) {
            PX4_ERR("SyncVel sub failed %i", res);
            return res;
        }

        res = _rate_sub.start(RateMessageCbBinder(this, &UavcanFmuSyncReceiver::rate_sub_cb));
        if (res < 0) {
            PX4_ERR("SyncRate sub failed %i", res);
            return res;
        }

        return 0;
    }

private:

    typedef uavcan::MethodBinder < UavcanFmuSyncReceiver *,
            void (UavcanFmuSyncReceiver::*)(const uavcan::ReceivedDataStructure<uavcan::kaiken::SyncVel> &) >
            VelMessageCbBinder;

    typedef uavcan::MethodBinder < UavcanFmuSyncReceiver *,
            void (UavcanFmuSyncReceiver::*)(const uavcan::ReceivedDataStructure<uavcan::kaiken::SyncRate> &) >
            RateMessageCbBinder;

    /**
    * Velocity sync message will be reported via this callback.
    */
    void velocity_sub_cb(const uavcan::ReceivedDataStructure<uavcan::kaiken::SyncVel> &msg)
    {
        integral_part_velocitycontrol_s _velocity_uorb_topic{};

        /*PX4_INFO("Received SyncVel with x=%f y=%f z=%f!",
                 (double)msg.velocity_integral[0], (double)msg.velocity_integral[1], (double)msg.velocity_integral[2]);*/

        _velocity_uorb_topic.timestamp = hrt_absolute_time();
        _velocity_uorb_topic.x = msg.velocity[0];
        _velocity_uorb_topic.y = msg.velocity[1];
        _velocity_uorb_topic.z = msg.velocity[2];
        _velocity_pub.publish(_velocity_uorb_topic);
    }

    /**
    * Rate sync message will be reported via this callback.
    */
    void rate_sub_cb(const uavcan::ReceivedDataStructure<uavcan::kaiken::SyncRate> &msg)
    {
        integral_part_ratecontrol_s _rate_uorb_topic{};

        /*PX4_INFO("Received SyncVel with x=%f y=%f z=%f!",
                 (double)msg.velocity_integral[0], (double)msg.velocity_integral[1], (double)msg.velocity_integral[2]);*/

        _rate_uorb_topic.timestamp = hrt_absolute_time();
        _rate_uorb_topic.x = msg.rate[0];
        _rate_uorb_topic.y = msg.rate[1];
        _rate_uorb_topic.z = msg.rate[2];
        _rate_pub.publish(_rate_uorb_topic);
    }

    /*
     * Publish
     */
    uORB::Publication<integral_part_velocitycontrol_s> _velocity_pub{ORB_ID(integral_part_velocitycontrol)};
    uORB::Publication<integral_part_ratecontrol_s>     _rate_pub{ORB_ID(integral_part_ratecontrol)};

    /*
     * Subscribe
     */
    uavcan::Subscriber<uavcan::kaiken::SyncVel, VelMessageCbBinder>   _velocity_sub;
    uavcan::Subscriber<uavcan::kaiken::SyncRate, RateMessageCbBinder> _rate_sub;
};
