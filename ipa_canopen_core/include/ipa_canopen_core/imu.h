#ifndef IMU_H
#define IMU_H

#include <ipa_canopen_core/device.h>
#include <boost/shared_ptr.hpp>

using namespace canopen;

class Imu : public Device
{
public:
    int16_t roll, pitch;
    Imu(uint8_t CANid, std::string name):
        Device(CANid, name, "imu"),
        roll(0),
        pitch(0) {}
    void TPDO1_incoming(const TPCANRdMsg m);
    void init_pdo();
    void set_objects();
};

typedef boost::shared_ptr<Imu> ImuPtr;

ImuPtr as_imu(DevicePtr ptr);

#endif
