#include <ipa_canopen_core/motor.h>
#include <algorithm>
#include <ros/ros.h>

void Motor::init_pdo()
{
    incomingPDOHandlers[ COB_PDO1_TX + CANid_ ] = [this](const TPCANRdMsg m) { TPDO1_incoming( m ); };
    incomingPDOHandlers[ COB_EMERGENCY + CANid_ ] = [this](const TPCANRdMsg m) { error_cb( m ); };

    int read_int;
    n_p->param(prefix_ + name + "/motor_params/has_encoder", read_int, 1);
    has_encoder = read_int;

    // check if analog inputs are supposed to be used
    if(n_p->getParam(prefix_ + name + "/analog_params/publish_inhibit_time", read_int))
    {
        use_analog = true;
        analog0_inhibit_time = (double)read_int/1000.0;
        n_p->param(prefix_ + name + "/analog_params/filter_depth", read_int, 20);
        analog0.resize(read_int);
        incomingPDOHandlers[ COB_PDO3_TX + CANid_ ] = [this](const TPCANRdMsg m) { TPDO3_incoming( m ); };
        ROS_INFO_STREAM("Publishing analog values for motor " << name << " with filter depth " << analog0.size());
    }

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

void Motor::set_objects()
{
    // Set Profile Acceleration
    set_sdos(ObjectKey(0x6083,0x00,0x20), prefix_ + name + "/drive_params/acceleration");

    // Set Profile Deceleration
    set_sdos(ObjectKey(0x6084,0x00,0x20), prefix_ + name + "/drive_params/deceleration");

    // Load gear ratio
    n_p->param(prefix_ + name + "/drive_params/ticks_per_rad_or_meter", joint_state.ticks_per_rad_or_meter, 4096);

    // Set Max Acceleration
    set_sdos(ObjectKey(0x60c5,0x00,0x20), prefix_ + name + "/drive_params/max_acceleration");

    // Set Max Deceleration
    set_sdos(ObjectKey(0x60c6,0x00,0x20), prefix_ + name + "/drive_params/max_deceleration");

    // Closed Loop Control
    set_sdos(ObjectKey(0x3202,0x00,0x20), prefix_ + name + "/motor_params/motor_drive_submode");

    // Set Pole Pairs
    set_sdos(ObjectKey(0x2030,0x00,0x20), prefix_ + name + "/motor_params/pole_pairs");

    // Set Encoder Resolution
    set_sdos(ObjectKey(0x2052,0x00,0x20),  prefix_ + name + "/motor_params/encoder_resolution");
    set_sdos(ObjectKey(0x608F,0x01,0x20), prefix_ + name + "/motor_params/encoder_resolution");

    // Set Peak Current [mA]
    set_sdos(ObjectKey(0x2031,0x00,0x20), prefix_ + name + "/motor_params/peak_current");

    // Set Peak Current Maximum Duration [ms]
    set_sdos(ObjectKey(0x203B,0x02,0x20), prefix_ + name + "/motor_params/peak_duration");

    // Set Nominal Current [mA]
    set_sdos(ObjectKey(0x203B,0x01,0x20), prefix_ + name + "/motor_params/nominal_current");

    // Set Max Torque [1/1000 * peak current]
    set_sdos(ObjectKey(0x6072,0x00,0x10), prefix_ + name + "/drive_params/max_torque");

    // Set Torque Slope
    set_sdos(ObjectKey(0x6087,0x00,0x20), prefix_ + name + "/drive_params/torque_slope");

    // Set Alignment
    set_sdos(ObjectKey(0x2050,0x00,0x20), prefix_ + name + "/motor_params/alignment");
    set_sdos(ObjectKey(0x2050,0x00,0x20), prefix_ + name + "/alignment");

    // Set Encoder Configuration (Voltage select or switch between single-ended or diff encoder
    set_sdos(ObjectKey(0x2059, 0x00,0x20), prefix_ + name + "/motor_params/encoder_configuration");

    n_p->param<double>(prefix_ + name + "/drive_params/jerk", max_jerk, 1000.0);

    // check if user program is used
    n_p->param<std::string>(prefix_ + name + "/user_code", user_code, "none");
    if(user_code == "jerk_pdo")
    {
        sendSDO(ObjectKey(0x2300,0x00,0x20), 1);
        ROS_WARN("Starting User Code Jerk");
    }
    else
    {
        // Set S-Curve position profile and limit jerk
        sendSDO(ObjectKey(0x60A4,1,0x20), max_jerk);
        sendSDO(ObjectKey(0x60A4,2,0x20), max_jerk);

        // work-around for inconsistent jerk SDOs
        int max_pdos;
        n_p->param(prefix_ + name + "/motor_params/pdo_channels", max_pdos, 4);
        if(max_pdos == 4)
        {
            sendSDO(ObjectKey(0x60A4,3,0x20), max_jerk);
            sendSDO(ObjectKey(0x60A4,4,0x20), max_jerk);
        }
    }
    if(user_code == "flyref")
    {
        if(set_sdos(ObjectKey(0x2400,2,0x20), prefix_ + name + "/flyref_ticks_per_mark") == false)
        {
            ROS_ERROR_STREAM("You must set parameter motors/" << name << "/flyref_ticks_per_mark if using flyref");
            return;
        }
        sendSDO(ObjectKey(0x2300,0x00,0x20), 1);
        incomingPDOHandlers[ COB_PDO4_TX + CANid_ ] = [this](const TPCANRdMsg m) { TPDO4_incoming( m ); };
        ROS_WARN("Starting User Code FlyRef");
    }
    else
    {
        incomingPDOHandlers[ COB_PDO2_TX + CANid_ ] = [this](const TPCANRdMsg m) { TPDO2_incoming( m ); };
    }

    set_sdos(ObjectKey(0x6086,0,0x10), prefix_ + name + "/drive_params/motion_profile");

    // Compensate Polepair Count
    set_sdos(ObjectKey(0x2060,0x00,0x20), prefix_ + name + "/drive_params/polepair_compensation");

    // Set PID Parameters
    set_sdos(ObjectKey(0x3210,0x01,0x20), prefix_ + name + "/control_params/position_p");
    set_sdos(ObjectKey(0x3210,0x02,0x20), prefix_ + name + "/control_params/position_i");
    set_sdos(ObjectKey(0x3210,0x03,0x20), prefix_ + name + "/control_params/velocity_p");
    set_sdos(ObjectKey(0x3210,0x04,0x20), prefix_ + name + "/control_params/velocity_i");
    set_sdos(ObjectKey(0x3210,0x05,0x20), prefix_ + name + "/control_params/current_field_p");
    set_sdos(ObjectKey(0x3210,0x06,0x20), prefix_ + name + "/control_params/current_field_i");
    set_sdos(ObjectKey(0x3210,0x07,0x20), prefix_ + name + "/control_params/current_torque_p");
    set_sdos(ObjectKey(0x3210,0x08,0x20), prefix_ + name + "/control_params/current_torque_i");

    // Profile Position Tolerance Window
    set_sdos(ObjectKey(0x6067,0,0x20), prefix_ + name + "/control_params/position_tolerance");

    set_sdos(ObjectKey(0x3240,0x01,0x20), prefix_ + name + "/homing_params/special_ios");
    set_sdos(ObjectKey(0x3240,0x02,0x20), prefix_ + name + "/homing_params/inverted_ios");
    set_sdos(ObjectKey(0x6098,0x00,0x08), prefix_ + name + "/homing_params/homing_method");
    set_sdos(ObjectKey(0x6099,0x01,0x20), prefix_ + name + "/homing_params/homing_speed");

    set_sdos(ObjectKey(0x2010,0x01,0x08), prefix_ + name + "/homing_params/input_pin");
    set_sdos(ObjectKey(0x2010,0x02,0x08), prefix_ + name + "/homing_params/input_pin_inverted");

    // Quick Stop Action
    set_sdos(ObjectKey(0x605A,0x00,0x10), prefix_ + name + "/drive_params/quick_stop_action");

    // Check polarity (turning direction of motor)
    set_sdos(ObjectKey(0x607E,0x00,0x08), prefix_ + name + "/polarity");
    int pol;
    n_p->param(prefix_ + name + "/polarity", pol, 1);
    polarity = ((pol>>7)&&1)?-1:1; // set polarity to -1 if 7th bit is set

    // internal limits
    // set_sdos(ObjectKey(0x607d,0x01), prefix_ + name + "/soft_limit_min");
    // set_sdos(ObjectKey(0x607b,0x02), prefix_ + name + "/soft_limit_max");

    // home offset in ticks
    set_sdos(ObjectKey(0x607C,0x00,0x20), prefix_ + name + "/home_offset");

    // maximal percentage of current in open loop
    set_sdos(ObjectKey(0x2004,0x01,0x08), prefix_ + name + "/motor_params/max_current_percentage");

    // Maximum current in percent for old nanotec drives
    set_sdos(ObjectKey(0x2004,0x01,0x08), prefix_ + name + "/drive_params/max_current_percent");

    // Reduced current when motor is not moving for new nanotec drives
    set_sdos(ObjectKey(0x2037,0x00,0x20), prefix_ + name + "/drive_params/reduced_current");

    // Enable closed loop mode for old nanotec drives
    set_sdos(ObjectKey(0x2001,0x00,0x08), prefix_ + name + "/motor_params/enable_closed_loop");

    // Brake setting
    set_sdos(ObjectKey(0x2038,0x01,0x20), prefix_ + name + "/drive_params/brake_activation_delay_ms");
    set_sdos(ObjectKey(0x2038,0x02,0x20), prefix_ + name + "/drive_params/brake_current_deactivation_delay_ms");
    set_sdos(ObjectKey(0x2038,0x03,0x20), prefix_ + name + "/drive_params/brake_deactivation_delay_ms");
    set_sdos(ObjectKey(0x2038,0x04,0x20), prefix_ + name + "/drive_params/brake_current_activation_delay_ms");

    // Limit Switch Tolerance
    set_sdos(ObjectKey(0x2056,0x00,0x20), prefix_ + name + "/limit_switch_tolerance");

    // Define upper voltage after which over voltage error should be triggered
    set_sdos(ObjectKey(0x2034, 0x00,0x20), prefix_ + name + "/motor_params/overvoltage");
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
    ROS_ERROR_STREAM("Todo: write comprehensive error message");
}

MotorPtr as_motor(DevicePtr ptr)
{
    return boost::static_pointer_cast<Motor>(ptr);
}
