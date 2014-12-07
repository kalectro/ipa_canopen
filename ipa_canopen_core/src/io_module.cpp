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
    incomingPDOHandlers[ COB_PDO1_TX + CANid_ ] = [this](const TPCANRdMsg m) { TPDO1_incoming( m ); };

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

void IoModule::set_objects()
{
    // Enable Input Interrupts
    // Load list of "any change" interrupts from param server
    XmlRpc::XmlRpcValue any_changes;
    if(n_p->getParam("io_modules/" + get_name() + "/io_interrupts/any_change", any_changes) == false)
    {
        ROS_ERROR_STREAM("Could not find parameter io_modules/" << get_name() << "/io_interrupts/any_change");
        return;
    }
    // Compose transmit message through bit shifting
    uint64_t any_change_transmit = 0;
    for(int i = 0; i < any_changes.size(); ++i)
    {
        any_change_transmit += (1 << any_changes[i]);
    }
    // any_change_transmit now contains up to 64 any change interrupt activations
    // now split up in groups of 8 bits
    for(int i=0; i<8; ++i)
    {
        uint64_t temp = 0xFF << (8*i); // create 8 bit block
        uint8_t transmit_byte = (any_change_transmit & temp) >> (8*i); // mask and shift back to lowest byte
        if(transmit_byte) // only transmit if not zero
        {
            sendSDO(ObjectKey(0x6006, i+1, 0x08), transmit_byte);
        }
    }

    // Global enable interrupts
    sendSDO(ObjectKey(0x6005,0x00, 0x08), 1);
}

IoModulePtr as_io_module(DevicePtr ptr)
{
    return boost::static_pointer_cast<IoModule>(ptr);
}
