#ifndef DEVICE_GROUP_H
#define DEVICE_GROUP_H

#include <ipa_canopen_core/canopen.h>
#include <ipa_canopen_core/device.h>
#include <ipa_canopen_core/motor.h>
#include <ipa_canopen_core/encoder.h>
#include <ipa_canopen_core/imu.h>
#include <ipa_canopen_core/io_module.h>
#include <boost/shared_ptr.hpp>

using namespace canopen;

class DeviceGroup
{
private:
    std::string name_;
    std::map<std::string, DevicePtr> devices_;
public:
    bool get_device(std::string name, DevicePtr& return_device);
    std::vector<DevicePtr> get_devices();

    DeviceGroup(std::string name):
        name_(name) {}

    MotorPtr add_motor(uint8_t CANid, std::string motor_name);
    IoModulePtr add_io_module(uint8_t CANid, std::string io_name);
    ImuPtr add_imu(uint8_t CANid, std::string imu_name);
    EncoderPtr add_encoder(uint8_t CANid, std::string encoder_name);
};

extern std::map<uint8_t, DevicePtr> device_id_map;

#endif
