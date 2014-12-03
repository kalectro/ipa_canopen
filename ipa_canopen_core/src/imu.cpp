#include <ipa_canopen_core/imu.h>


ImuPtr as_imu(DevicePtr ptr)
{
    return boost::static_pointer_cast<Imu>(ptr);
}

void Imu::TPDO1_incoming(const TPCANRdMsg m)
{
    pitch = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8);
    roll = m.Msg.DATA[2] + (m.Msg.DATA[3] << 8);
}

void Imu::init_pdo()
{
    uint8_t tsync_type, rsync_type;
    for(int pdo_channel = 1; pdo_channel <=4 ; pdo_channel++)
    {
        tpdo_registers_.clear();
        rpdo_registers_.clear();
        switch(pdo_channel)
        {
            case 1:
                tpdo_registers_.push_back(ObjectKey(0x6010, 0, 2));  // longitudinal [0.01°]
                tpdo_registers_.push_back(ObjectKey(0x6020, 0, 2));  // lateral [0.01°]

                tsync_type = SYNC_TYPE_MANUFACTURER_SPECIFIC;
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            default:
                ROS_ERROR("There are only 4 PDO channels");
                return;
                break;
        }
        pdo_map(pdo_channel, tsync_type, rsync_type);
    }
}
