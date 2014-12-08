#ifndef ENCODER_H
#define ENCODER_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ipa_canopen_core/device.h>
#include <sensor_msgs/JointState.h>

using namespace canopen;

class Encoder : public Device
{
public:
    sensor_msgs::JointState joint_state;
    Encoder(uint8_t CANid, std::string name):
        Device(CANid, name, "encoder"),
        joint_state(),
        ticks_per_rad_or_meter_(4096)
    {
        joint_state.name.push_back(name);
        joint_state.position.push_back(0.0);
        joint_state.velocity.push_back(0.0);
    }

    void TPDO1_incoming(const TPCANRdMsg m);
    void init_pdo();
    void set_objects();

private:
    double ticks_per_rad_or_meter_;
};

typedef boost::shared_ptr<Encoder> EncoderPtr;

EncoderPtr as_encoder(DevicePtr ptr);

#endif
