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
    incomingPDOHandlers[ COB_PDO1_TX + CANid_ ] = [this](const TPCANRdMsg m) { TPDO1_incoming( m ); };

    uint8_t tsync_type, rsync_type;
    for(int pdo_channel = 1; pdo_channel <=1 ; pdo_channel++)
    {
        tpdo_registers_.clear();
        rpdo_registers_.clear();
        switch(pdo_channel)
        {
            case 1:
                tpdo_registers_.push_back(ObjectKey(0x6010, 0, 16));  // longitudinal [0.01°]
                tpdo_registers_.push_back(ObjectKey(0x6020, 0, 16));  // lateral [0.01°]

                tsync_type = SYNC_TYPE_MANUFACTURER_SPECIFIC;
                break;
            default:
                ROS_ERROR("There are only 1 PDO channels");
                return;
                break;
        }
        pdo_map(pdo_channel, tsync_type, rsync_type);
    }
}

void Imu::set_objects()
{
    // Set Filter Type
    set_sdos(ObjectKey(0x3000,0x01,0x10), "imu_modules/" + get_name() + "/filter_type");

    // Set Filter Frequency
    set_sdos(ObjectKey(0x3000,0x02,0x10), "imu_modules/" + get_name() + "/filter_frequency");
}
