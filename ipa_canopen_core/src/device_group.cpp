#include <ipa_canopen_core/device_group.h>

MotorPtr DeviceGroup::add_motor(uint8_t CANid, std::string motor_name)
{
    MotorPtr new_device( new Motor(CANid, motor_name) );
    devices_[motor_name] = new_device;
    device_id_map[CANid] = new_device;
    return new_device;
}

IoModulePtr DeviceGroup::add_io_module(uint8_t CANid, std::string io_name)
{
    IoModulePtr new_device( new IoModule(CANid, io_name) );
    devices_[io_name] = new_device;
    device_id_map[CANid] = new_device;
    return new_device;
}

ImuPtr DeviceGroup::add_imu(uint8_t CANid, std::string imu_name)
{
    ImuPtr new_device( new Imu(CANid, imu_name) );
    devices_[imu_name] = new_device;
    device_id_map[CANid] = new_device;
    return new_device;
}

EncoderPtr DeviceGroup::add_encoder(uint8_t CANid, std::string encoder_name)
{
    EncoderPtr new_device( new Encoder(CANid, encoder_name) );
    devices_[encoder_name] = new_device;
    device_id_map[CANid] = new_device;
    return new_device;
}

std::vector<DevicePtr> DeviceGroup::get_devices()
{
    std::vector<DevicePtr> return_vector;

    for( auto it : devices_)
    {
        return_vector.push_back( it.second );
    }
    return return_vector;
}

bool DeviceGroup::get_device(std::string name, DevicePtr& return_device)
{
    if(devices_.find(name) != devices_.end())
    {
        return_device = devices_[name];
        return true;
    }
    ROS_WARN_STREAM("Did not find device with name " << name << " in DeviceGroup " << name_);
    return false;
}
