#include <ipa_canopen_core/device.h>
#include <ipa_canopen_core/io_module.h>

void IoModule::TPDO1_incoming(const TPCANRdMsg m)
{
    inputs = 0;
    for(int i=0; i<8; ++i)
    {
        inputs += ((uint64_t)m.Msg.DATA[i]) << (8*i);
    }
}

void IoModule::init_pdo()
{
    uint8_t tsync_type, rsync_type;
    for(int pdo_channel = 1; pdo_channel <=4 ; pdo_channel++)
    {
        tpdo_registers_.clear();
        rpdo_registers_.clear();
        switch(pdo_channel)
        {
            case 1:
                tpdo_registers_.push_back(ObjectKey(0x6000, 1, 1));  // DI0..7
                tpdo_registers_.push_back(ObjectKey(0x6000, 2, 1));  // DI8..15
                tpdo_registers_.push_back(ObjectKey(0x6000, 3, 1));  // DI16..23

                tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                rsync_type = SYNC_TYPE_ASYNCHRONOUS;
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

IoModulePtr as_io_module(DevicePtr ptr)
{
    return boost::static_pointer_cast<IoModule>(ptr);
}
