#include <ipa_canopen_core/device.h>

void Device::uploadSDO(ObjectKey sdo)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid_ + COB_SDO_RX;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0x40;
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write_debug(h, &msg);
}

bool Device::sendSDO(ObjectKey sdo, int value, bool verify, int32_t trials, double timeout)
{
    // build SDO message
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid_ + COB_SDO_RX;
    msg.LEN = 4 + (sdo.size / 8);
    // Bit 5 = download request, Bit 2-3 = not used bytes, Bit 1 = expedited, Bit 0 = size indicated
    msg.DATA[0] = (1 << 5) + ((4-(sdo.size / 8)) << 2) + (1 << 1) + (1 << 0);
    msg.DATA[1] = sdo.index & 0xFF;
    msg.DATA[2] = (sdo.index >> 8) & 0xFF;
    msg.DATA[3] = sdo.subindex;
    msg.DATA[4] = value & 0xFF;
    msg.DATA[5] = (value >> 8) & 0xFF;
    msg.DATA[6] = (value >> 16) & 0xFF;
    msg.DATA[7] = (value >> 24) & 0xFF;

    int32_t trial_counter = 0;
    response_sdo.can_id = 0;
    response_sdo.object.index = 0; // make sure no old values are used
    response_sdo.object.subindex = 0;
    response_sdo.value = 0;

    static ros::Time start = ros::Time::now();
    while(response_sdo.can_id != CANid_ || response_sdo.object.index != sdo.index || response_sdo.object.subindex != sdo.subindex || response_sdo.value != value)  // possible problem here! Previous: (IntType)response_sdo.value != value
    {
        // Send new value
        CAN_Write_debug(h, &msg);

        if(verify == false)
            return true;

        // Check if value was written
        uploadSDO(sdo);

        // tiemout?
        if((ros::Time::now() - start).toSec() > timeout / (double)trials)
        {
            if(trial_counter++ >= trials)
            {
                std::stringstream error;
                ROS_ERROR_STREAM("Write error at CANid 0x" << std::hex << (int)CANid_ << " to SDO " << sdo.index << "s" << (int)sdo.subindex <<" with value " << (int)value << ", read value " << response_sdo.value);
                return false;
            }
            // Restart timer
            start = ros::Time::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}

void Device::disableRPDO(int object)
{
    int32_t data;
    switch(object)
    {
        case 0:
            data = (RPDO1_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
            break;
        case 1:
            data = (RPDO2_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
            break;
        case 2:
            data = (RPDO3_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
            break;
        case 3:
            data = (RPDO4_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
            break;
        default:
            std::cout << "BAD OBJECT NUMBER IN disableRPDO! Number is " << object << std::endl;
            return;
    }
    sendSDO(ObjectKey(RPDO+object,1,32), data);
}

void Device::clearRPDOMapping(int object)
{
    sendSDO(ObjectKey(RPDO_map+object,0,8), 0);
}

void Device::makeRPDOMapping(int object, uint8_t sync_type)
{
    int counter;
    for(counter=0; counter < rpdo_registers_.size();counter++)
    {
        uint32_t data = rpdo_registers_[counter].size + (rpdo_registers_[counter].subindex << 8) + (rpdo_registers_[counter].index << 16);
        sendSDO(ObjectKey(RPDO_map + object, counter + 1, 32), data);
    }

    sendSDO(ObjectKey(RPDO+object, 2, 8), sync_type);
    ROS_DEBUG_STREAM("Mapping " << std::hex << counter << " objects at CANid " << (int)CANid_ << " to RPDO" << object + 1);
    sendSDO(ObjectKey(RPDO_map+object,0,8), counter);
}

void Device::enableRPDO(int object)
{
    int32_t data;
    switch(object)
    {
        case 0:
            data = COB_PDO1_RX + CANid_;
            break;
        case 1:
            data = COB_PDO2_RX + CANid_;
            break;
        case 2:
            data = COB_PDO3_RX + CANid_;
            break;
        case 3:
            data = COB_PDO4_RX + CANid_;
            break;
        default:
            ROS_ERROR("Wrong object number in enableRPDO");
            return;
    }
    sendSDO(ObjectKey(RPDO+object, 1, 32), data);
}

void Device::disableTPDO(int object)
{
    int32_t data;
    switch(object)
    {
        case 0:
            data = (TPDO1_msg + CANid_)  + (1 << 31);
            break;
        case 1:
            data = (TPDO2_msg + CANid_)  + (1 << 31);
            break;
        case 2:
            data = (TPDO3_msg + CANid_)  + (1 << 31);
            break;
        case 3:
            data = (TPDO4_msg + CANid_)  + (1 << 31);
            break;
        default:
            std::cout << "Incorrect object for mapping" << std::endl;
            return;
    }
    sendSDO(ObjectKey(TPDO+object,1,32), data);
}

void Device::clearTPDOMapping(int object)
{
    sendSDO(ObjectKey(TPDO_map+object,0,8), 0);
}

void Device::makeTPDOMapping(int object, uint8_t sync_type)
{
    int counter;
    for(counter = 0; counter < tpdo_registers_.size(); counter++)
    {
        uint32_t data = tpdo_registers_[counter].size + (tpdo_registers_[counter].subindex << 8) + (tpdo_registers_[counter].index << 16);
        sendSDO(ObjectKey(TPDO_map + object, counter + 1, 32), data);
    }

    sendSDO(ObjectKey(TPDO+object,2,8), sync_type);
    ROS_DEBUG_STREAM("Mapping " << std::hex << counter << " objects at CANid " << (int)CANid_ << " to TPDO" << object + 1);
    sendSDO(ObjectKey(TPDO+object,3,16), 10);

    if(device_type == "imu" || device_type == "encoder") // send cyclic every 10ms
    {
        sendSDO(ObjectKey(TPDO+object,5,16), 10);
    }
    sendSDO(ObjectKey(TPDO_map+object,0,8), counter);
}

void Device::enableTPDO(int object)
{
    int32_t data;
    switch(object)
    {
        case 0:
            data = COB_PDO1_TX + CANid_;
            break;
        case 1:
            data = COB_PDO2_TX + CANid_;
            break;
        case 2:
            data = COB_PDO3_TX + CANid_;
            break;
        case 3:
            data = COB_PDO4_TX + CANid_;
            break;
        default:
            std::cout << "Incorrect object number handed over to enableTPDO" << std::endl;
            return;
    }
    sendSDO(ObjectKey(TPDO+object,1,32), data);
}

void Device::pdo_map(int pdo_id, uint8_t tsync_type, uint8_t rsync_type)
{
    // clear all mappings for given pdo id
    disableTPDO(pdo_id-1);
    clearTPDOMapping(pdo_id-1);
    if(!tpdo_registers_.empty())
    {
        makeTPDOMapping(pdo_id-1, tsync_type);
        enableTPDO(pdo_id-1);
    }
    if(!rpdo_registers_.empty())
    {
        disableRPDO(pdo_id-1);
        clearRPDOMapping(pdo_id-1);
        makeRPDOMapping(pdo_id-1, rsync_type);
        enableRPDO(pdo_id-1);
    }
}

bool Device::init(std::string deviceFile)
{
    if(canopen::atFirstInit)
    {
        canopen::atFirstInit = false;

        if(std::find(canopen::openDeviceFiles.begin(), canopen::openDeviceFiles.end(), deviceFile) == canopen::openDeviceFiles.end())
        {
            CAN_Close(canopen::h);

            if (!canopen::openConnection(deviceFile, canopen::baudRate)) // check if connection was successful
            {
                ROS_ERROR_STREAM("Cannot open CAN device " << deviceFile << "... aborting.");
                exit(EXIT_FAILURE);
            }
            canopen::initListenerThread(canopen::defaultListener);
            canopen::openDeviceFiles.push_back(deviceFile);
        }
        sendNMT(NMT_RESET_COMMUNICATION, true);
    }

    ros::Time start = ros::Time::now();

    while(!nmt_init)
    {
        if((ros::Time::now() - start).toSec() > 5.0)
        {
            ROS_ERROR_STREAM("Node: " << (int)CANid_ << " is not ready for operation. Please check for potential problems.");
            return false;
        }

        ros::Duration(0.01).sleep();
    }

    // Configure PDO channels
    ROS_DEBUG_STREAM("mapping PDOs of device " << name);
    init_pdo();

    ros::Duration(0.01).sleep();
    sendNMT(NMT_START_REMOTE_NODE);
    // std::cout << std::hex << "Initialized the PDO mapping for Node: " << (int)CANid << std::endl;
    initialized = true;

    set_objects();

    return true;
}

void Device::sdo_incoming(BYTE data[8])
{
    // data[0] -> command byte
    uint16_t sdo_id = data[1]+(data[2]<<8);
    uint8_t sdo_id_sub = data[3];
    response_sdo.can_id = CANid_;
    response_sdo.object.index = sdo_id;
    response_sdo.object.subindex = sdo_id_sub;
    response_sdo.aborted = false;
    response_sdo.confirmed = false;

    // read out data
    if(data[0] & 0x03)  // expedited transfer
    {
        // check bit 2 and 3 for unused bytes
        switch((data[0] & 0x0C) >> 2)
        {
            case 0:
                response_sdo.value = data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24);
                break;
            case 1:
                response_sdo.value = data[4] + (data[5] << 8) + (data[6] << 16);
                break;
            case 2:
                response_sdo.value = data[4] + (data[5] << 8);
                break;
            case 3:
                response_sdo.value = data[4];
                break;
        }
    }
    else if(data[0] == 0x60)  // SDO confirm
    {
        response_sdo.confirmed = true;
        //std::cout << std::hex << "Write SDO confirmed from " << (int)CANid << " with id " << sdo_id << "s" << (int)sdo_id_sub << std::endl;
    }
    else if(data[0] == 0x80)  // SDO abort
    {
        response_sdo.aborted = true;
        uint32_t abort_code = data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24);
        // Ignore Nanotec bug message
        if(abort_code == 0x06040042)
            return;
        auto iter = sdo_abort_messages.find(abort_code);
        std::string error_message = "SDO Abort";
        if ( iter != sdo_abort_messages.end())
        {
            error_message = (*iter).second;
        }
        ROS_ERROR_STREAM("SDO abort from CAN id " << std::hex << (int)CANid_ << " for SDO 0x" << sdo_id << "s" << (int)sdo_id_sub << " with the following error message: " << error_message);
        return;
    }
    else // no idea what I received
    {
        ROS_WARN_STREAM("Received SDO from 0x" << std::hex << (int)CANid_ << " with id 0x" << sdo_id << "s" << (int)sdo_id_sub << " and command byte 0x" << (int)data[0] << "  DATA: " << (int)data[4] << " " << (int)data[5] << " " << (int)data[6] << " " << (int)data[7]);
    }

    // check if SDO was requested
    if(sdo_id == requested_sdo.object.index && sdo_id_sub == requested_sdo.object.subindex)
    {
        //std::cout << "requested sdo received " << response_sdo.value << std::endl;
        requested_sdo.confirmed = true;
        requested_sdo.value = response_sdo.value;
    }
}

void Device::nmt_incoming(BYTE data[8])
{
    if(nmt_state.find(data[0])->second == "Bootup")
    {
        // catch second bootup message after device was initialized
        if(nmt_init && initialized)
        {
            ROS_ERROR_STREAM("RECEIVED SECOND BOOTUP FROM CAN ID " << (int)CANid_ << "  THIS IS BAD!");
        }
        else
        {
            nmt_init = true;
            ROS_INFO_STREAM("Bootup from CANid " << std::hex << (int)CANid_);
        }
    }
}

bool Device::set_sdos(ObjectKey sdo, std::string param)
{
    int temp;
    if (n_p->getParam(param, temp))
    {
        if(sendSDO(sdo, temp))
            return true;
        else
            return false;
    }
    else
    {
        ROS_DEBUG_STREAM("Param " << param << " is not available...");
        return false;
    }
}

void Device::sendNMT(uint8_t command, bool send_to_all)
{
    TPCANMsg NMTmsg;
    std::memset(&NMTmsg, 0, sizeof(NMTmsg));
    NMTmsg.ID = 0;
    NMTmsg.MSGTYPE = 0x00;
    NMTmsg.LEN = 2;
    NMTmsg.DATA[0] = command;
    if(send_to_all)
        NMTmsg.DATA[1] = 0;
    else
        NMTmsg.DATA[1] = CANid_;
    CAN_Write_debug(h, &NMTmsg);
}

Device::Device(uint8_t CANid, std::string name, std::string type):
    CANid_(CANid),
    initialized(false),
    device_type(type),
    inputs(0), outputs_(0),
    nmt_init(false),
    name(name) {}
