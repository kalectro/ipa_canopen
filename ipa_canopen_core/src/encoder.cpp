#include <ipa_canopen_core/encoder.h>

void Encoder::TPDO1_incoming(const TPCANRdMsg m)
{
    int32_t ticks = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16);
    int16_t ticks_per_sec = m.Msg.DATA[4] + (m.Msg.DATA[5] << 8);
    ticks <<= 8;  // inflate 24 Bit signed int to 32 Bit including
    ticks /= 64;  // revert inflating
    joint_state.position[0] = (double)ticks         / ticks_per_rad_or_meter_;
    joint_state.velocity[0] = (double)ticks_per_sec / ticks_per_rad_or_meter_;
    joint_state.header.stamp = ros::Time::now();
}

void Encoder::init_pdo()
{
    incomingPDOHandlers[ COB_PDO1_TX + CANid_ ] = [this](const TPCANRdMsg m) { TPDO1_incoming( m ); };

    uint8_t tsync_type, rsync_type;
    for(int pdo_channel = 1; pdo_channel <=2 ; pdo_channel++)
    {
        tpdo_registers_.clear();
        rpdo_registers_.clear();
        switch(pdo_channel)
        {
            case 1:
                tpdo_registers_.push_back(ObjectKey(0x6004, 0, 4));  // Actual Position
                tpdo_registers_.push_back(ObjectKey(0x6030, 1, 2));  // Actual Velocity

                tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                break;
            case 2:
                break;
            default:
                ROS_ERROR("ERROR: There are only 2 PDO channels");
                return;
                break;
        }
        pdo_map(pdo_channel, tsync_type, rsync_type);
    }
}

void Encoder::set_objects()
{
    // Load gear ratio
    n_p->param("encoders/" + get_name() + "/ticks_per_rad_or_meter", ticks_per_rad_or_meter_, 4096.0);

    // Activate velocity measurements
    sendSDO( ObjectKey(0x3010,0x01,0x08), 1);
}

EncoderPtr as_encoder(DevicePtr ptr)
{
    return boost::static_pointer_cast<Encoder>(ptr);
}
