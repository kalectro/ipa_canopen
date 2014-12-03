/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2013 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   Project name: ipa_canopen
 * \note
 *   ROS stack name: ipa_canopen
 * \note
 *   ROS package name: ipa_canopen_core
 *
 * \author
 *   Author: Thiago de Freitas, Tobias Sing, Eduard Herkel
 * \author
 *   Supervised by: Thiago de Freitas email:tdf@ipa.fhg.de
 *
 * \date Date of creation: December 2012
 *
 * \brief
 *   Implementation of canopen driver.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ipa_canopen_core/canopen.h>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <ros/ros.h>

namespace canopen
{
    /***************************************************************/
    //			define global variables and functions
    /***************************************************************/
    void (*error_handler)(const std::string&) = NULL;

    std::string baudRate;
    bool canbus_error = false;
    HANDLE h;
    std::vector<std::string> openDeviceFiles;
    bool atFirstInit=true;
    SDOanswer requested_sdo;
    SDOanswer response_sdo;

    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;

    std::string operation_mode_param;

    ros::Time start, end;
    ros::Duration elapsed_seconds;

    std::map<uint8_t, DevicePtr> device_id_map;
    std::map<std::string, DevicePtr> device_name_map;

    /***************************************************************/
    //		define init sequence
    /***************************************************************/

    bool openConnection(std::string devName, std::string baudrate)
    {
        h = LINUX_CAN_Open(devName.c_str(), O_RDWR);
        if (!h)
            return false;

        errno = CAN_Init(h, baudrates[baudrate], CAN_INIT_TYPE_ST);

        return true;
    }

    DWORD CAN_Write_debug(HANDLE h, TPCANMsg *msg)
    {
        if(canbus_error)
        {
            return -1;
        }
        DWORD status = CAN_Status(h);
        int nreads, nwrites;
        int counter = 0;
        while(status & 0x80)
        {
            status = LINUX_CAN_Extended_Status(h, &nreads, &nwrites);
            std::stringstream error;
            error << "ugly status... waiting... for " << nreads << " reads and " << nwrites << " writes";
            output_error(error.str());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if(counter++ >10)
            {
                output_error("STATUS UGLY... ABORTING CAN WRITE");
                canbus_error = true;
                return -1;
            }
        }
        DWORD return_me = LINUX_CAN_Write_Timeout(h, msg, 10000);
        if(return_me != 0)
            std::cout << "WRITE ERROR " << return_me << std::endl;
        return return_me;
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

    void Motor::init_pdo(int pdo_channel)
    {
        uint8_t tsync_type, rsync_type;
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

    DevicePtr DeviceGroup::add_device(uint8_t CANid, std::string motor_name)
    {
        DevicePtr new_device( new Device(CANid, motor_name) );
        device_id_map[CANid] = new_device;
        return new_device;
    }

    bool Device::init(std::string deviceFile, uint8_t max_pdo_channels)
    {
        if(canopen::atFirstInit)
        {
            canopen::atFirstInit = false;

            if(std::find(canopen::openDeviceFiles.begin(), canopen::openDeviceFiles.end(), deviceFile) == canopen::openDeviceFiles.end())
            {
                CAN_Close(canopen::h);

                if (!canopen::openConnection(deviceFile, canopen::baudRate)) // check if connection was successful
                {
                    output_error("Cannot open CAN device " + deviceFile + "; aborting.");
                    exit(EXIT_FAILURE);
                }
                canopen::initListenerThread(canopen::defaultListener);
                canopen::openDeviceFiles.push_back(deviceFile);
            }
            sendNMT(0x00, canopen::NMT_RESET_COMMUNICATION);
        }

        start = ros::Time::now();

        while(!nmt_init)
        {
            end = ros::Time::now();
            elapsed_seconds = end - start;

            if(elapsed_seconds.toSec() > 10.0)
            {
                std::stringstream error;
                error << "Node: " << CANid_ << " is not ready for operation. Please check for potential problems." << std::endl;
                output_error(error.str());
                return false;
            }

            ros::Duration(0.01).sleep();
        }

        // std::cout << "Node: " << (uint16_t)id << " is now available" << std::endl;

        // Configure PDO channels
        for (int pdo_channel = 1; pdo_channel <= max_pdo_channels; ++pdo_channel)
        {
            u_int8_t tsync_type, rsync_type;
            if(is_io_module)
            {
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
                        std::cout << "ERROR: There are only 4 PDO channels" << std::endl;
                        return false;
                        break;
                }
            }
            else if(is_imu)
            {
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
                        std::cout << "ERROR: There are only 4 PDO channels" << std::endl;
                        return false;
                        break;
                }
            }
            else if(is_encoder)
            {
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
                        std::cout << "ERROR: There are only 2 PDO channels" << std::endl;
                        return false;
                        break;
                }
            }
            pdo_map(pdo_channel, tsync_type, rsync_type);
        }

        ros::Duration(0.01).sleep();
        sendNMT(CANid_, canopen::NMT_START_REMOTE_NODE);
        // std::cout << std::hex << "Initialized the PDO mapping for Node: " << (int)CANid << std::endl;
        initialized = true;
        return true;
    }

    void sendNMT(uint8_t CANid, uint8_t command)
    {
        TPCANMsg NMTmsg;
        std::memset(&NMTmsg, 0, sizeof(NMTmsg));
        NMTmsg.ID = 0;
        NMTmsg.MSGTYPE = 0x00;
        NMTmsg.LEN = 2;
        NMTmsg.DATA[0] = command;
        NMTmsg.DATA[1] = CANid;
        CAN_Write_debug(h, &NMTmsg);
    }

    void output_error(std::string incoming_error)
    {
        if(error_handler)
        {
            error_handler(incoming_error);
        }
        else
        {
            std::cout << incoming_error << std::endl;
        }
    }

    bool Motor::setOperationMode(int8_t targetMode, double timeout)
    {
        start = ros::Time::now();

        // check if motor is in a legitimate state to change operation mode
        if (    state != MS_READY_TO_SWITCH_ON &&
                state != MS_SWITCHED_ON_DISABLED &&
                state != MS_SWITCHED_ON)
        {
            ROS_DEBUG_STREAM("Found motor " << name << " in state " << state << ", adjusting to SWITCHED_ON");
            setMotorState(canopen::MS_SWITCHED_ON);
        }

        // std::cout << std::dec << "setting mode of motor " << (int)CANid << " to " << modesDisplay.find(targetMode)->second << std::endl;
        operation_mode_target_ = targetMode;
        this->controlPDO();
        // check operation mode until correct mode is returned
        while (this->status.actual_operation_mode != targetMode)
        {
            // timeout check
            end = ros::Time::now();
            elapsed_seconds = end-start;

            if(elapsed_seconds.toSec() > timeout)
            {
                output_error("setting operation mode failed");
                return false;
            }
        }

        return true;
    }

    bool Motor::setMotorState(std::string targetState, double timeout)
    {
        start = ros::Time::now();
        // std::cout << "Setting state of motor " << (int)CANid << " to " << targetState << std::endl;
        while (state != targetState)
        {
            end = ros::Time::now();
            elapsed_seconds = end-start;

            if(elapsed_seconds.toSec() > timeout)
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
            else
            {
                ROS_ERROR_STREAM("I do not know how to change from state " << state << " to state " << targetState);
                return false;
            }
            ros::Duration(0.01).sleep();  // give motor some time to allow changing state
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

    void IoModule::TPDO1_incoming(const TPCANRdMsg m)
    {
        inputs = 0;
        for(int i=0; i<8; ++i)
        {
            inputs += ((uint64_t)m.Msg.DATA[i]) << (8*i);
        }
    }

    void Imu::TPDO1_incoming(const TPCANRdMsg m)
    {
        pitch = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8);
        roll = m.Msg.DATA[2] + (m.Msg.DATA[3] << 8);
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

        joint_state.position = (double)ticks / joint_state.ticks_per_rad_or_meter * polarity;
        joint_state.velocity = (double)ticks_per_sec / joint_state.ticks_per_rad_or_meter * polarity;
        joint_state.stamp = ros::Time::now();
    }

    void Motor::TPDO4_incoming(const TPCANRdMsg m)
    {
        int32_t ticks = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16) + (m.Msg.DATA[3] << 24);
        joint_state.position = (double)ticks / joint_state.ticks_per_rad_or_meter * polarity;
        joint_state.stamp = ros::Time::now();

        nanoj_outputs = m.Msg.DATA[4] + (m.Msg.DATA[5] << 8) + (m.Msg.DATA[6] << 16) + (m.Msg.DATA[7] << 24);
    }

    void Encoder::TPDO1_incoming(const TPCANRdMsg m)
    {
        int32_t ticks = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16);
        int16_t ticks_per_sec = m.Msg.DATA[4] + (m.Msg.DATA[5] << 8);
        ticks <<= 8;  // inflate 24 Bit signed int to 32 Bit including
        ticks /= 64;  // revert inflating
        joint_state.position = (double)ticks / joint_state.ticks_per_rad_or_meter * polarity;
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

        start = ros::Time::now();
        while(response_sdo.can_id != CANid_ || response_sdo.object.index != sdo.index || response_sdo.object.subindex != sdo.subindex || response_sdo.value != value)  // possible problem here! Previous: (IntType)response_sdo.value != value
        {
            // Send new value
            CAN_Write_debug(h, &msg);

            if(verify == false)
                return true;

            // Check if value was written
            uploadSDO(sdo);

            //if(response_sdo.index == sdo.index && response_sdo.subindex == sdo.subindex && response_sdo.aborted == true)
            //{
            //    std::cout << std::hex << "SDO request aborted by CANid " << (int)CANid << " for SDO " << sdo.index << "s" << (int)sdo.subindex <<" with value " << (int)value << std::endl;
            //    return false;
            //}
            end = ros::Time::now();
            elapsed_seconds = end - start;
            if(elapsed_seconds.toSec() > timeout / (double)trials)
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

    bool DeviceGroup::get_device(std::string name, DevicePtr return_device)
    {
        if(devices_.find(name) != devices_.end())
        {
            return true;
            return_device = devices_[name];
        }
        ROS_WARN_STREAM("Did not find device with name " << name << " in DeviceGroup " << name_);
        return false;
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

    void EMCY_incoming(uint8_t CANid, const TPCANRdMsg m)
    {
        std::stringstream error_stream;
        uint16_t error_code =  m.Msg.DATA[0] +  (m.Msg.DATA[1] << 8);
        uint8_t error_class = m.Msg.DATA[2];
        uint8_t error_number = m.Msg.DATA[3];


        error_stream << std::hex << "ERROR from CANid " << (int)CANid << ": " << NanotecErrorNumber[error_number] << "   ERROR CATEGORIES: ";

        if ( error_class & EMC_k_1001_GENERIC )
            error_stream << "generic ";
        if ( error_class & EMC_k_1001_CURRENT)
            error_stream << "current ";
        if ( error_class & EMC_k_1001_VOLTAGE )
            error_stream << "voltage ";
        if ( error_class & EMC_k_1001_TEMPERATURE )
            error_stream << "temperature ";
        if ( error_class & EMC_k_1001_COMMUNICATION )
            error_stream << "communication ";
        if ( error_class & EMC_k_1001_DEV_PROF_SPEC )
            error_stream << "device profile specific ";
        if ( error_class & EMC_k_1001_RESERVED )
            error_stream << "reserved ";
        if ( error_class & EMC_k_1001_MANUFACTURER)
            error_stream << "manufacturer specific ";

        auto iter = error_codes.find(error_code);
        error_stream << " Error Code: ";
        if ( iter != error_codes.end())
        {
            error_stream << iter->second;
        }
        else
        {
            error_stream << std::hex << error_code;
        }

        ROS_ERROR_STREAM(error_stream);
    }

    void initListenerThread(std::function<void ()> const& listener)
    {
        std::thread listener_thread(listener);
        listener_thread.detach();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //std::cout << "Listener thread initialized" << std::endl;
    }

    void defaultListener()
    {
        while(!canbus_error)
        {
            //std::cout << "Reading incoming data" << std::endl;
            TPCANRdMsg m;
            errno = LINUX_CAN_Read(h, &m);
            if(errno != 0)
                output_error("LINUX_CAN_Read()");
            if (errno)
                perror("LINUX_CAN_Read() error");

            // incoming SYNC
            else if (m.Msg.ID == COB_SYNC)
            {
            }

            // incoming EMCY
            else if (m.Msg.ID >= COB_EMERGENCY && m.Msg.ID < COB_TIME_STAMP)
            {
                EMCY_incoming(m.Msg.ID - COB_EMERGENCY, m);
                if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                    incomingPDOHandlers[m.Msg.ID](m);
            }

            // incoming TIME
            else if (m.Msg.ID == COB_TIME_STAMP)
            {
            }

            // incoming PD0
            else if (m.Msg.ID >= COB_PDO1_TX && m.Msg.ID < COB_PDO4_RX)
            {
                if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                    incomingPDOHandlers[m.Msg.ID](m);
            }

            // incoming SD0
            else if (m.Msg.ID >= COB_SDO_TX && m.Msg.ID < COB_SDO_RX)
            {
                sdo_incoming(m.Msg.ID - COB_SDO_TX, m.Msg.DATA);
            }

            // incoming NMT heartbeat
            else if (m.Msg.ID >= COB_NODEGUARD && m.Msg.ID < COB_MAX)
            {
                nmt_incoming(m.Msg.ID - COB_NODEGUARD, m.Msg.DATA);
            }

            else
            {
                std::stringstream error;
                error << "Received unknown message with id 0x" << std::hex << m.Msg.ID << " and Data 0x" << (int)m.Msg.DATA[0] << " 0x" << (int)m.Msg.DATA[1] << " 0x" << (int)m.Msg.DATA[2] << " 0x" << (int)m.Msg.DATA[3];
                output_error(error.str());
            }
        }
    }

    void sdo_incoming(uint8_t CANid, BYTE data[8])
    {
        // data[0] -> command byte
        uint16_t sdo_id = data[1]+(data[2]<<8);
        uint8_t sdo_id_sub = data[3];
        response_sdo.can_id = CANid;
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
            std::stringstream error_text;
            error_text << std::hex << "SDO abort from CAN id " << (int)CANid << " for SDO 0x" << sdo_id << "s" << (int)sdo_id_sub << " with the following error message: " << error_message << std::endl;
            output_error(error_text.str());
        }
        else // no idea what I received
        {
            std::cout << std::hex << "Received SDO from 0x" << (int)CANid << " with id 0x" << sdo_id << "s" << (int)sdo_id_sub << " and command byte 0x" << (int)data[0] << "  DATA: " << (int)data[4] << " " << (int)data[5] << " " << (int)data[6] << " " << (int)data[7] << std::endl;
        }

        // check if SDO was requested
        if(sdo_id == requested_sdo.object.index && sdo_id_sub == requested_sdo.object.subindex && CANid == requested_sdo.can_id)
        {
            //std::cout << "requested sdo received " << response_sdo.value << std::endl;
            requested_sdo.confirmed = true;
            requested_sdo.value = response_sdo.value;
        }
    }

    void nmt_incoming(uint8_t CANid, BYTE data[8])
    {
        std::map<uint8_t,DevicePtr>::const_iterator search = device_id_map.find(CANid);
        if(search != device_id_map.end())
        {
            DevicePtr device;
            // std::cout << "Found " << (u_int16_t)search->first << "\n";
            if(nmt_state.find(data[0])->second == "Bootup")
            {
                // catch second bootup message after device was initialized
                if(device->nmt_init && device->initialized)
                {
                    std::stringstream error;
                    error << "RECEIVED SECOND BOOTUP FROM CAN ID " << (int)CANid << "  THIS IS BAD!";
                    output_error(error.str());
                }
                else
                {
                    device->nmt_init = true;
                    std::cout << std::hex << "Bootup from CANid " << (int)CANid << std::endl;
                }
            }
        }
        else
        {
            std::cout << "Received bootup from node " << (int)CANid << " which I do not know what to do with...ignoring" << std::endl;
        }
    }

    void Device::disableRPDO(int object)
    {
        int32_t data;
        switch(object)
        {
            case 0:
                data = (canopen::RPDO1_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 1:
                data = (canopen::RPDO2_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 2:
                data = (canopen::RPDO3_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 3:
                data = (canopen::RPDO4_msg + CANid_)  + (0x00 << 16) + (0x80 << 24);
                break;
            default:
                std::cout << "BAD OBJECT NUMBER IN disableRPDO! Number is " << object << std::endl;
                return;
        }
        sendSDO(ObjectKey(RPDO.index+object,0x01), data);
    }

    void Device::clearRPDOMapping(int object)
    {
        sendSDO(ObjectKey(RPDO_map.index+object,0x00), (uint8_t)0x00);
    }

    void Device::makeRPDOMapping(int object, uint8_t sync_type)
    {
        int counter;
        for(counter=0; counter < rpdo_registers_.size();counter++)
        {
            uint32_t data = rpdo_registers_[counter].size + (rpdo_registers_[counter].subindex << 8) + (rpdo_registers_[counter].index << 16);
            sendSDO(ObjectKey(RPDO_map.index + object, counter + 1), data);
        }

        sendSDO(ObjectKey(RPDO.index+object, 0x02), uint8_t(sync_type));
        ROS_DEBUG_STREAM("Mapping " << std::hex << counter << " objects at CANid " << (int)CANid_ << " to RPDO" << object + 1);
        sendSDO(ObjectKey(RPDO_map.index+object,0x00), uint8_t(counter));
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
        sendSDO(ObjectKey(RPDO.index+object, 0x01), data);
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
        sendSDO(ObjectKey(TPDO.index+object,0x01), data);
    }

    void Device::clearTPDOMapping(int object)
    {
        sendSDO(ObjectKey(TPDO_map.index+object,0x00), uint8_t(0x00));
    }

    void Device::makeTPDOMapping(int object, uint8_t sync_type)
    {
        int counter;
        for(counter = 0; counter < tpdo_registers_.size(); counter++)
        {
            uint32_t data = tpdo_registers_[counter].size + (tpdo_registers_[counter].subindex << 8) + (tpdo_registers_[counter].index << 16);
            sendSDO(ObjectKey(TPDO_map.index + object, counter + 1), data);
        }

        sendSDO(ObjectKey(TPDO.index+object,0x02), uint8_t(sync_type));
        ROS_DEBUG_STREAM("Mapping " << std::hex << counter << " objects at CANid " << (int)CANid_ << " to TPDO" << object + 1);
        sendSDO(ObjectKey(TPDO.index+object,0x03), uint16_t(10));

        if(is_imu || is_encoder)  // send cyclic every 10ms
        {
            sendSDO(ObjectKey(TPDO.index+object,0x05), uint16_t(10));
        }
        sendSDO(ObjectKey(TPDO_map.index+object,0x00), uint8_t(counter));
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
        sendSDO(ObjectKey(TPDO.index+object,0x01), data);
    }

    void set_error_handler(void (*set_me)(const std::string&))
    {
        error_handler = set_me;
    }

    MotorPtr as_motor(DevicePtr ptr)
    {
        return boost::static_pointer_cast<Motor>(ptr);
    }

    ImuPtr as_imu(DevicePtr ptr)
    {
        return boost::static_pointer_cast<Imu>(ptr);
    }

    IoModulePtr as_io_module(DevicePtr ptr)
    {
        return boost::static_pointer_cast<IoModule>(ptr);
    }

    EncoderPtr as_encoder(DevicePtr ptr)
    {
        return boost::static_pointer_cast<Encoder>(ptr);
    }

}
