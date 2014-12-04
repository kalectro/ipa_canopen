#include <ipa_canopen_core/motor.h>
#include <algorithm>
#include <ros/ros.h>

void Motor::init_pdo()
{
    uint8_t tsync_type, rsync_type;
    for(int pdo_channel = 1; pdo_channel <=4 ; pdo_channel++)
    {
        tpdo_registers_.clear();
        rpdo_registers_.clear();
        switch(pdo_channel)
        {
            case 1:
                tpdo_registers_.push_back(ObjectKey(0x6041, 0, 0x10));  // Status word
                tpdo_registers_.push_back(ObjectKey(0x6061, 0, 0x08));  // Mode of operation display
                tpdo_registers_.push_back(ObjectKey(0x60FD, 0, 0x20));  // Digital Inputs

                rpdo_registers_.push_back(ObjectKey(0x6040, 0, 0x10));  // Control word
                rpdo_registers_.push_back(ObjectKey(0x60FE, 1, 0x20));  // Digital Outputs
                rpdo_registers_.push_back(ObjectKey(0x6060, 0, 0x08));  // Mode of operation

                tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                break;
            case 2:
                if(this->has_encoder)
                {
                    tpdo_registers_.push_back(ObjectKey(0x6064, 0, 0x20));  // Position Actual Value
                    tpdo_registers_.push_back(ObjectKey(0x6044, 0, 0x10));  // Velocity Actual Value
                }
                else
                {
                    tpdo_registers_.push_back(ObjectKey(0x6062, 0, 0x20));  // Position Demand Value
                    tpdo_registers_.push_back(ObjectKey(0x6043, 0, 0x10));  // Velocity Demand Value
                }

                rpdo_registers_.push_back(ObjectKey(0x607A, 0, 0x20));  // Target Position Value
                rpdo_registers_.push_back(ObjectKey(0x6081, 0, 0x20));  // Profile Velocity

                tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                break;
            case 3:
                if(this->use_analog)
                {
                    tpdo_registers_.push_back(ObjectKey(0x3220, 1, 0x10));  // Analog Input Channel 0
                    tpdo_registers_.push_back(ObjectKey(0x3220, 2, 0x10));  // Analog Input Channel 1
                }

                rpdo_registers_.push_back(ObjectKey(0x6083, 0, 0x20));  // Profile Acceleration
                rpdo_registers_.push_back(ObjectKey(0x6084, 0, 0x20));  // Profile Deceleration

                tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                break;
            case 4:
                tpdo_registers_.push_back(ObjectKey(0x2500, 1, 0x20));  // NanoJ Outputs 2500:1
                tpdo_registers_.push_back(ObjectKey(0x2500, 2, 0x20));  // NanoJ Outputs 2500:2

                rpdo_registers_.push_back(ObjectKey(0x6071, 0, 0x10));  // Target Torque
                rpdo_registers_.push_back(ObjectKey(0x2400, 1, 0x20)); // VMM Inputs 2400:1

                tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                break;
        }
        pdo_map(pdo_channel, tsync_type, rsync_type);
    }
}

bool Motor::setOperationMode(int8_t targetMode, double timeout)
{
    static ros::Time start;
    start = ros::Time::now();

    // check if motor is in a legitimate state to change operation mode
    if (    state != MS_READY_TO_SWITCH_ON &&
            state != MS_SWITCHED_ON_DISABLED &&
            state != MS_SWITCHED_ON)
    {
        ROS_DEBUG_STREAM("Found motor " << name << " in state " << state << ", adjusting to SWITCHED_ON");
        setMotorState(canopen::MS_SWITCHED_ON);
    }

    // check operation mode until correct mode is returned
    while (status.actual_operation_mode != targetMode)
    {
        // timeout check
        if((ros::Time::now() - start).toSec() > timeout)
        {
            ROS_ERROR_STREAM("setting operation mode for motor " << name << " failed");
            return false;
        }
        operation_mode_target_ = targetMode;
        controlPDO();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // give motor some time to allow changing state
    }

    return true;
}

bool Motor::setMotorState(std::string targetState, double timeout)
{
    static ros::Time start;
    start = ros::Time::now();
    ROS_DEBUG_STREAM("Setting state of motor " << (int)CANid_ << " to " << targetState);
    while (state != targetState)
    {
        if((ros::Time::now() - start).toSec() > timeout)
            return false;
        if(state == MS_FAULT)
        {
            if(!status.fault)
            {
                sendControlWord(CONTROLWORD_FAULT_RESET_0);
            }
            else
            {
                sendControlWord(CONTROLWORD_FAULT_RESET_1);
            }
        }
        else if(state == MS_NOT_READY_TO_SWITCH_ON)
        {
            sendControlWord(CONTROLWORD_SHUTDOWN);
        }
        else if(state == MS_SWITCHED_ON_DISABLED)
        {
            sendControlWord(CONTROLWORD_SHUTDOWN);
        }
        else if(state == MS_READY_TO_SWITCH_ON)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                sendControlWord(CONTROL_WORD_DISABLE_VOLTAGE);
            }
            else
            {
                sendControlWord(CONTROLWORD_SWITCH_ON);
            }
        }
        else if(state == MS_SWITCHED_ON)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                sendControlWord(CONTROL_WORD_DISABLE_VOLTAGE);
            }
            else if (targetState == MS_READY_TO_SWITCH_ON)
            {
                sendControlWord(CONTROLWORD_SHUTDOWN);
            }
            else
            {
                sendControlWord(CONTROLWORD_ENABLE_OPERATION);
            }
        }
        else if(state == MS_OPERATION_ENABLED)
        {
            if (targetState == MS_SWITCHED_ON_DISABLED)
            {
                sendControlWord(CONTROL_WORD_DISABLE_VOLTAGE);
            }
            else if (targetState == MS_READY_TO_SWITCH_ON)
            {
                sendControlWord(CONTROLWORD_SHUTDOWN);
            }
            else
            {
                sendControlWord(CONTROLWORD_DISABLE_OPERATION);
            }
        }
        else if(state == MS_START_UP)
        {
            ROS_INFO_STREAM("Motor " << name << " did not send a statusword yet...");
            uploadSDO(STATUSWORD);
        }
        else
        {
            ROS_ERROR_STREAM("I do not know how to change from state " << state << " to state " << targetState);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // give motor some time to allow changing state
    }
    return true;
}

void Motor::sendControlWord(uint16_t target_controlword)
{
    status.controlword = target_controlword;
    controlPDO();
}

void Motor::controlPDO()
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = CANid_ + COB_PDO1_RX;
    msg.MSGTYPE = 0x00;
    msg.LEN = 7;
    msg.DATA[0] = status.controlword;
    msg.DATA[1] = status.controlword >> 8;
    msg.DATA[2] = 0;
    msg.DATA[3] = 0;
    msg.DATA[4] = outputs_;
    msg.DATA[5] = outputs_ >> 8;
    msg.DATA[6] = operation_mode_target_;

    // std::cout << "controlPDO: controlword 0x" << std::hex << devices[CANid].controlword << "  operation_mode " << (int)devices[CANid].operation_mode_target << std::endl;
    CAN_Write_debug(h, &msg);
}

void Motor::setOutputs(uint64_t target_outputs)
{
    outputs_ = target_outputs;
    controlPDO();
}

bool Motor::check_operation_mode(int8_t target_mode)
{
    int8_t current_mode = status.actual_operation_mode;
    if(current_mode != target_mode)
    {
        //
        // ROS_WARN_STREAM("Motor is in mode " << modesDisplay.find(current_mode)->second << " instead of " << modesDisplay.find(target_mode)->second << "... changing");
        // ROS_WARN_STREAM("Motor is in mode " << modesDisplay[current_mode] << " instead of " << modesDisplay[target_mode] << "... changing");
        uploadSDO(STATUSWORD);
        if(setOperationMode(target_mode) == false)
        {
            ROS_ERROR("Could not set operation mode :(");
            return false;
        }
        if (!setMotorState(MS_OPERATION_ENABLED))
        {
            ROS_ERROR_STREAM("setMotorState of " << name << " to MS_OPERATION_ENABLED failed");
            return false;
        }
    }
    return true;
}

void Motor::RPDO4_torque(int16_t target_torque)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = COB_PDO4_RX + CANid_;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = target_torque & 0xFF;
    msg.DATA[1] = (target_torque >> 8) & 0xFF;
    msg.DATA[2] = 0x00;
    msg.DATA[3] = 0x00;
    msg.DATA[4] = 0x00;
    msg.DATA[5] = 0x00;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write_debug(h, &msg);
}

void Motor::TPDO1_incoming(const TPCANRdMsg m)
{
    uint16_t mydata_low = m.Msg.DATA[0];
    uint16_t mydata_high = m.Msg.DATA[1];

    status.statusword = mydata_low + (mydata_high << 8);
    status.actual_operation_mode = m.Msg.DATA[2];

    inputs = m.Msg.DATA[5] + (m.Msg.DATA[6] << 8);
    status.ready_switch_on = mydata_low & 0x01;
    status.switched_on = mydata_low & 0x02;
    status.op_enable = mydata_low & 0x04;
    status.fault = mydata_low & 0x08;
    status.volt_enable = mydata_low & 0x10;
    status.quick_stop = mydata_low & 0x20;
    status.switch_on_disabled = mydata_low & 0x40;
    status.warning = mydata_low & 0x80;

    status.mode_specific = mydata_high & 0x01;
    status.remote = mydata_high & 0x02;
    status.target_reached = mydata_high & 0x04;
    status.internal_limit = mydata_high & 0x08;
    status.op_specific0 = mydata_high & 0x10;
    status.op_specific1 = mydata_high & 0x20;
    status.man_specific0 = mydata_high & 0x40;
    status.man_specific1 = mydata_high & 0x80;

    // check motor state by bit masking statusword
    switch(status.statusword & 0b1101111)
    {
        case 0b0000000: // fall through
        case 0b0100000: state = MS_NOT_READY_TO_SWITCH_ON; break;
        case 0b1000000: // fall through
        case 0b1100000: state = MS_SWITCHED_ON_DISABLED; break;
        case 0b0100001: state = MS_READY_TO_SWITCH_ON; break;
        case 0b0100011: state = MS_SWITCHED_ON; break;
        case 0b0100111: state = MS_OPERATION_ENABLED; break;
        case 0b0000111: state = MS_QUICK_STOP_ACTIVE; break;
        case 0b0001111: // fall through
        case 0b0101111: state = MS_FAULT_REACTION_ACTIVE; break;
        case 0b0001000: // fall through
        case 0b0101000: state = MS_FAULT; break;
        default: ROS_ERROR_STREAM("UNKNOWN MOTOR STATE FROM MOTOR " << name);
    }

    // check if target was just reached
    if(status.target_reached == false && (mydata_high & 0x04) == 0x04)
    {
        ROS_DEBUG_STREAM("Motor " << name << " reached its target at " << ros::Time::now() << " seconds");
    }
}

void Motor::TPDO2_incoming(const TPCANRdMsg m)
{
    int32_t ticks = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16) + (m.Msg.DATA[3] << 24);
    int16_t ticks_per_sec = m.Msg.DATA[4] + (m.Msg.DATA[5] << 8);

    joint_state.position = (double)ticks         / joint_state.ticks_per_rad_or_meter * polarity;
    joint_state.velocity = (double)ticks_per_sec / joint_state.ticks_per_rad_or_meter * polarity;
    joint_state.stamp = ros::Time::now();
}

void Motor::TPDO3_incoming(const TPCANRdMsg m)
{
    if(analog0.size() > 0)
    {
        analog0.pop_front();  // delete oldest value
    }
    if(analog1.size() > 0)
    {
        analog1.pop_front();  // delete oldest value
    }
    analog0.push_back(m.Msg.DATA[0] + (m.Msg.DATA[1] << 8));
    analog1.push_back(m.Msg.DATA[2] + (m.Msg.DATA[3] << 8));
}

void Motor::TPDO4_incoming(const TPCANRdMsg m)
{
    int32_t ticks = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16) + (m.Msg.DATA[3] << 24);
    joint_state.position = (double)ticks / joint_state.ticks_per_rad_or_meter * polarity;
    joint_state.stamp = ros::Time::now();

    nanoj_outputs = m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24);
}

void Motor::RPDO2_profile_position(int32_t target_position, uint32_t max_velocity)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = COB_PDO2_RX + CANid_;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = target_position & 0xFF;
    msg.DATA[1] = (target_position >> 8) & 0xFF;
    msg.DATA[2] = (target_position >> 16) & 0xFF;
    msg.DATA[3] = (target_position >> 24) & 0xFF;
    msg.DATA[4] = max_velocity & 0xFF;
    msg.DATA[5] = (max_velocity >> 8) & 0xFF;
    msg.DATA[6] = (max_velocity >> 16) & 0xFF;
    msg.DATA[7] = (max_velocity >> 24) & 0xFF;
    CAN_Write_debug(h, &msg);
}

void Motor::RPDO4_jerk(uint32_t profile_jerk)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = COB_PDO4_RX + CANid_;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0;
    msg.DATA[1] = 0;
    msg.DATA[2] = profile_jerk & 0xFF;
    msg.DATA[3] = (profile_jerk >> 8) & 0xFF;
    msg.DATA[4] = (profile_jerk >> 16) & 0xFF;
    msg.DATA[5] = (profile_jerk >> 24) & 0xFF;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write_debug(h, &msg);
}

void Motor::RPDO4_position_rectified(int32_t profile_position)
{
    TPCANMsg msg;
    std::memset(&msg, 0, sizeof(msg));
    msg.ID = COB_PDO4_RX + CANid_;
    msg.MSGTYPE = 0x00;
    msg.LEN = 8;
    msg.DATA[0] = 0;
    msg.DATA[1] = 0;
    msg.DATA[2] = profile_position & 0xFF;
    msg.DATA[3] = (profile_position >> 8) & 0xFF;
    msg.DATA[4] = (profile_position >> 16) & 0xFF;
    msg.DATA[5] = (profile_position >> 24) & 0xFF;
    msg.DATA[6] = 0x00;
    msg.DATA[7] = 0x00;
    CAN_Write_debug(h, &msg);
}

void Motor::error_cb(const TPCANRdMsg m)
{
    ROS_ERROR_STREAM(" ERROR: " << last_error);
}

MotorPtr as_motor(DevicePtr ptr)
{
    return boost::static_pointer_cast<Motor>(ptr);
}
