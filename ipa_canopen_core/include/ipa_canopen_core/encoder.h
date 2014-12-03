#ifndef ENCODER_H
#define ENCODER_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ipa_canopen_core/device.h>

using namespace canopen;

class Encoder : public Device
{
public:
    JointState joint_state;
    Encoder(uint8_t CANid, std::string name):
        Device(CANid, name, "encoder"),
        joint_state() {}

    void TPDO1_incoming(const TPCANRdMsg m);
    void init_pdo();
};

typedef boost::shared_ptr<Encoder> EncoderPtr;

EncoderPtr as_encoder(DevicePtr ptr);

#endif
