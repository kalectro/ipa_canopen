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
#include <unordered_map>
#include <algorithm>

namespace canopen
{
    /***************************************************************/
    //			define global variables and functions
    /***************************************************************/
    bool sdo_protect=false;
    BYTE protect_msg[8];

    std::string baudRate;
    std::map<uint8_t, Device> devices;
    std::map<std::string, DeviceGroup> deviceGroups;
    HANDLE h;
    std::vector<std::thread> managerThreads;
    std::vector<std::string> openDeviceFiles;
    bool atFirstInit=true;
    std::map<SDOkey, std::function<void (uint8_t CANid, BYTE data[8])> > incomingManufacturerDetails { {MANUFACTURERHWVERSION, manufacturer_incoming}, {MANUFACTURERDEVICENAME, manufacturer_incoming}, {MANUFACTURERSOFTWAREVERSION, manufacturer_incoming} };
    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;

    std::string operation_mode_param;

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

    std::chrono::duration<double> elapsed_seconds;

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
        DWORD status = CAN_Status(h);
        int nreads, nwrites;
        int counter = 0;
        while(status & 0x80)
        {
            status = LINUX_CAN_Extended_Status(h, &nreads, &nwrites);
            // std::cout << "ugly status... waiting... for " << nreads << " reads and " << nwrites << " writes" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if(counter++ >3)
            {
                std::cout << "STATUS UGLY... ABORTING CAN WRITE" << std::endl;
                return -1;
            }
        }
        DWORD return_me = LINUX_CAN_Write_Timeout(h, msg, 10000);
        if(return_me != 0)
            std::cout << "WRITE ERROR " << return_me << std::endl;
        return return_me;
    }

    void pdo_map(uint8_t id, int pdo_id,
                 std::vector<std::string> tpdo_registers, std::vector<int> tpdo_sizes, u_int8_t tsync_type,
                 std::vector<std::string> rpdo_registers, std::vector<int> rpdo_sizes, u_int8_t rsync_type)
    {
        if(!tpdo_registers.empty())
        {
            // clear all mappings for given pdo id
            canopen::disableTPDO(id, pdo_id-1);
            canopen::clearTPDOMapping(id, pdo_id-1);
            canopen::makeTPDOMapping(id, pdo_id-1, tpdo_registers, tpdo_sizes, tsync_type);
            canopen::enableTPDO(id, pdo_id-1);
        }
        if(!rpdo_registers.empty())
        {
            canopen::disableRPDO(id, pdo_id-1);
            canopen::clearRPDOMapping(id, pdo_id-1);
            canopen::makeRPDOMapping(id,pdo_id-1, rpdo_registers, rpdo_sizes, rsync_type);
            canopen::enableRPDO(id, pdo_id-1);
        }
    }

    bool init(std::string deviceFile, std::string chainName, uint8_t CANid, uint8_t max_pdo_channels)
    {
        if(canopen::atFirstInit)
        {
            canopen::atFirstInit = false;

            bool connection_success;

            bool connection_is_available = std::find(canopen::openDeviceFiles.begin(), canopen::openDeviceFiles.end(), deviceFile) != canopen::openDeviceFiles.end();
            if(!connection_is_available)
            {
                CAN_Close(canopen::h);
                connection_success = canopen::openConnection(deviceFile, canopen::baudRate);

                if (!connection_success)
                {
                    std::cout << "Cannot open CAN device "<< deviceFile << "; aborting." << std::endl;
                    exit(EXIT_FAILURE);
                }
                canopen::initListenerThread(canopen::defaultListener);
                canopen::openDeviceFiles.push_back(deviceFile);
            }

            // std::cout << "Resetting communication with the devices " << std::endl;
            canopen::sendNMT(0x00, canopen::NMT_RESET_COMMUNICATION);

        }

        if(canopen::deviceGroups[chainName].getFirstInit())
        {
            std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;
            time_start = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_seconds;

            canopen::initDeviceManagerThread(chainName,canopen::deviceManager);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            std::cout << "Initializing " << chainName << std::endl;

            canopen::deviceGroups[chainName].setFirstInit(false);

            while(sdo_protect)
            {
                time_end = std::chrono::high_resolution_clock::now();
                elapsed_seconds = time_end - time_start;

                if(elapsed_seconds.count() > 5.0)
                {
                    std::cout << "not ready for operation. Probably due to communication problems with the Master." << std::endl;
                    return false;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;
        time_start = std::chrono::high_resolution_clock::now();

        bool nmt_init = devices[CANid].getNMTInit();
        // std::cout << "Waiting for Node: " << (uint16_t)id << " to become available" << std::endl;

        while(!nmt_init)
        {
            time_end = std::chrono::high_resolution_clock::now();
            elapsed_seconds = time_end - time_start;

            if(elapsed_seconds.count() > 10.0)
            {
                std::cout << "Node: " << (uint16_t)CANid << " is not ready for operation. Please check for potential problems." << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            nmt_init = devices[CANid].getNMTInit();
        }

        // std::cout << "Node: " << (uint16_t)id << " is now available" << std::endl;

        // Configure PDO channels
        for (int pdo_channel = 1; pdo_channel <= max_pdo_channels; ++pdo_channel)
        {
            std::vector<std::string> tpdo_registers, rpdo_registers;
            std::vector<int> tpdo_sizes, rpdo_sizes;
            u_int8_t tsync_type, rsync_type;
            if(devices[CANid].is_motor)
            {
                switch(pdo_channel)
                {
                    case 1:
                        // Status word
                        tpdo_registers.push_back("604100");
                        tpdo_sizes.push_back(0x10);

                        // Mode of operation display
                        tpdo_registers.push_back("606100");
                        tpdo_sizes.push_back(0x08);

                        // Digital Inputs
                        tpdo_registers.push_back("60FD00");
                        tpdo_sizes.push_back(0x20);

                        // Control word
                        rpdo_registers.push_back("604000");
                        rpdo_sizes.push_back(0x10);

                        tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        break;
                    case 2:
                        if(devices[CANid].has_encoder)
                        {
                            // Position Actual Value
                            tpdo_registers.push_back("606400");
                            tpdo_sizes.push_back(0x20);
                            // Velocity Actual Value
                            tpdo_registers.push_back("604400");
                            tpdo_sizes.push_back(0x10);
                        }
                        else
                        {
                            // Position Demand Value
                            tpdo_registers.push_back("606200");
                            tpdo_sizes.push_back(0x20);
                            // Velocity Demand Value
                            tpdo_registers.push_back("604300");
                            tpdo_sizes.push_back(0x10);
                        }

                        // Target Position Value
                        rpdo_registers.push_back("607A00");
                        rpdo_sizes.push_back(0x20);
                        rsync_type = SYNC_TYPE_ASYNCHRONOUS;

                        // Max Velocity
                        rpdo_registers.push_back("608100");
                        rpdo_sizes.push_back(0x20);

                        tsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        break;
                    case 3:
                        // Profile Acceleration
                        rpdo_registers.push_back("608300");
                        rpdo_sizes.push_back(0x20);

                        // Profile Deceleration
                        rpdo_registers.push_back("608400");
                        rpdo_sizes.push_back(0x20);

                        rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        break;
                    case 4:
                        // Target Torque
                        rpdo_registers.push_back("607100");
                        rpdo_sizes.push_back(0x10);

                        rsync_type = SYNC_TYPE_ASYNCHRONOUS;
                        break;
                }
            }
            else if(devices[CANid].is_io_module)
            {
                switch(pdo_channel)
                {
                    case 1:
                        // DI0..7
                        tpdo_registers.push_back("600001");
                        tpdo_sizes.push_back(0x08);

                        // DI8..15
                        tpdo_registers.push_back("600002");
                        tpdo_sizes.push_back(0x08);

                        // DI16..23
                        tpdo_registers.push_back("600003");
                        tpdo_sizes.push_back(0x08);

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
            pdo_map(CANid, pdo_channel, tpdo_registers, tpdo_sizes, tsync_type, rpdo_registers, rpdo_sizes, rsync_type);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        canopen::sendNMT(CANid, canopen::NMT_START_REMOTE_NODE);
        std::cout << std::hex << "Initialized the PDO mapping for Node: " << (int)CANid << std::endl;
        canopen::devices[CANid].setInitialized(true);
        return true;
    }

    void setNMTState(uint8_t CANid, std::string targetState)
    {

    }

    bool setOperationMode(uint8_t CANid, int8_t targetMode, double timeout)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;

        time_start = std::chrono::high_resolution_clock::now();

        // check if motor is in a legitimate state to change operation mode
        if (    devices[CANid].getMotorState() != MS_READY_TO_SWITCH_ON &&
                devices[CANid].getMotorState() != MS_SWITCHED_ON_DISABLED &&
                devices[CANid].getMotorState() != MS_SWITCHED_ON)
        {
            std::cout << "Found motor in state " << devices[CANid].getMotorState() << ", adjusting to SWITCHED_ON" << std::endl;
            setMotorState(CANid, canopen::MS_SWITCHED_ON);
        }

        std::cout << std::dec << "setting mode to " << (int)targetMode << std::endl;
        sendSDO(CANid, MODES_OF_OPERATION, targetMode);
        // check operation mode until correct mode is returned
        while (devices[CANid].getCurrentModeofOperation() != targetMode)
        {
            // timeout check
            time_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_seconds = time_end-time_start;

            if(elapsed_seconds.count() > timeout)
            {
                std::cout << "setting operation mode failed" << std::endl;
                return false;
            }
        }

        return true;
    }

    bool setMotorState(uint8_t CANid, std::string targetState, double timeout)
    {
        start = std::chrono::high_resolution_clock::now();

        while (devices[CANid].getMotorState() != targetState)
        {
            end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;

            if(elapsed_seconds.count() > timeout)
                return false;
            if(devices[CANid].getMotorState() == MS_FAULT)
            {
                if(!devices[CANid].getFault())
                {
                    canopen::controlPDO(CANid, canopen::CONTROLWORD_FAULT_RESET_0);
                }
                else
                {
                    canopen::controlPDO(CANid, canopen::CONTROLWORD_FAULT_RESET_1);
                }
            }
            else if(devices[CANid].getMotorState() == MS_NOT_READY_TO_SWITCH_ON)
            {
                canopen::uploadSDO(CANid, canopen::STATUSWORD);
                canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN);
            }
            else if(devices[CANid].getMotorState() == MS_SWITCHED_ON_DISABLED)
            {
                canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN);
            }
            else if(devices[CANid].getMotorState() == MS_READY_TO_SWITCH_ON)
            {
                if (targetState == MS_SWITCHED_ON_DISABLED)
                {
                    canopen::controlPDO(CANid, canopen::CONTROL_WORD_DISABLE_VOLTAGE);
                }
                else
                {
                    canopen::controlPDO(CANid, canopen::CONTROLWORD_SWITCH_ON);
                }
            }
            else if(devices[CANid].getMotorState() == MS_SWITCHED_ON)
            {
                if (targetState == MS_SWITCHED_ON_DISABLED)
                {
                    canopen::controlPDO(CANid, canopen::CONTROL_WORD_DISABLE_VOLTAGE);
                }
                else if (targetState == MS_READY_TO_SWITCH_ON)
                {
                    canopen::controlPDO(CANid, canopen::CONTROLWORD_SHUTDOWN);
                }
                else
                {
                    canopen::controlPDO(CANid, canopen::CONTROLWORD_ENABLE_OPERATION);
                }
            }
            else if(devices[CANid].getMotorState() == MS_OPERATION_ENABLED)
            {
                if (targetState == MS_SWITCHED_ON_DISABLED)
                {
                    sendSDO(CANid, CONTROLWORD, CONTROL_WORD_DISABLE_VOLTAGE);
                }
                else if (targetState == MS_READY_TO_SWITCH_ON)
                {
                    sendSDO(CANid, CONTROLWORD, CONTROLWORD_SHUTDOWN);
                }
                else
                {
                    sendSDO(CANid, CONTROLWORD, CONTROLWORD_DISABLE_OPERATION);
                }
            }
        }
        return true;
    }

    TPCANMsg NMTmsg;
    TPCANMsg syncMsg;

    void requestDataBlock1(uint8_t CANid)
    {
        TPCANMsg msg;
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = CANid + COB_SDO_RX;
        msg.MSGTYPE = 0x00;
        msg.LEN = 8;
        msg.DATA[0] = 0x60;
        msg.DATA[1] = 0x00;
        msg.DATA[2] = 0x00;
        msg.DATA[3] = 0x00;
        msg.DATA[4] = 0x00;
        msg.DATA[5] = 0x00;
        msg.DATA[6] = 0x00;
        msg.DATA[7] = 0x00;
        CAN_Write_debug(h, &msg);
    }

    void requestDataBlock2(uint8_t CANid)
    {
        TPCANMsg msg;
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = CANid + COB_SDO_RX;
        msg.MSGTYPE = 0x00;
        msg.LEN = 8;
        msg.DATA[0] = 0x70;
        msg.DATA[1] = 0x00;
        msg.DATA[2] = 0x00;
        msg.DATA[3] = 0x00;
        msg.DATA[4] = 0x00;
        msg.DATA[5] = 0x00;
        msg.DATA[6] = 0x00;
        msg.DATA[7] = 0x00;
        CAN_Write_debug(h, &msg);
    }

    void controlPDO(uint8_t CANid, u_int16_t control_word)
    {
        TPCANMsg msg;
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = CANid + COB_PDO1_RX;
        msg.MSGTYPE = 0x00;
        msg.LEN = 2;
        msg.DATA[0] = control_word;
        msg.DATA[1] = control_word >> 8;
        CAN_Write_debug(h, &msg);
    }

    void uploadSDO(uint8_t CANid, SDOkey sdo)
    {
        TPCANMsg msg;
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = CANid + COB_SDO_RX;
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

    void test_sdo_types()
    {
        sendSDO(0,SDOkey(0,0),(uint8_t)0,0,0);
        sendSDO(0,SDOkey(0,0),(int8_t)0,0,0);
        sendSDO(0,SDOkey(0,0),(uint16_t)0,0,0);
        sendSDO(0,SDOkey(0,0),(int16_t)0,0,0);
        sendSDO(0,SDOkey(0,0),(uint32_t)0,0,0);
        sendSDO(0,SDOkey(0,0),(int32_t)0,0,0);
    }

    template< class IntType >
    bool sendSDO(uint8_t CANid, SDOkey sdo, IntType value, bool verify, int32_t trials, double timeout)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;
        std::chrono::duration<double> elapsed_seconds;
        // build SDO message
        TPCANMsg msg;
        int size_of_value = sizeof(value);
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = CANid + COB_SDO_RX;
        msg.LEN = 4 + size_of_value;
        // Bit 5 = download request, Bit 2-3 = not used bytes, Bit 1 = expedited, Bit 0 = size indicated
        msg.DATA[0] = (1 << 5) + ((4-size_of_value) << 2) + (1 << 1) + (1 << 0);
        msg.DATA[1] = sdo.index & 0xFF;
        msg.DATA[2] = (sdo.index >> 8) & 0xFF;
        msg.DATA[3] = sdo.subindex;
        msg.DATA[4] = value & 0xFF;
        msg.DATA[5] = (value >> 8) & 0xFF;
        msg.DATA[6] = (value >> 16) & 0xFF;
        msg.DATA[7] = (value >> 24) & 0xFF;

        int32_t trial_counter = 0;
        response_sdo.can_id = 0;
        response_sdo.index = 0; // make sure no old values are used
        response_sdo.subindex = 0;
        response_sdo.value = 0;

        time_start = std::chrono::high_resolution_clock::now();
        while(response_sdo.can_id != CANid || response_sdo.index != sdo.index || response_sdo.subindex != sdo.subindex || (IntType)response_sdo.value != value)
        {
            // Send new value
            CAN_Write_debug(h, &msg);

            if(verify == false)
                return true;

            // Check if value was written
            uploadSDO(CANid,sdo);

            //if(response_sdo.index == sdo.index && response_sdo.subindex == sdo.subindex && response_sdo.aborted == true)
            //{
            //    std::cout << std::hex << "SDO request aborted by CANid " << (int)CANid << " for SDO " << sdo.index << "s" << (int)sdo.subindex <<" with value " << (int)value << std::endl;
            //    return false;
            //}
            time_end = std::chrono::high_resolution_clock::now();
            elapsed_seconds = time_end - time_start;
            if(elapsed_seconds.count() > timeout / (double)trials)
            {
                if(trial_counter++ >= trials)
                {
                    std::cout << std::hex << "Write error at CANid " << (int)CANid << " to SDO " << sdo.index << "s" << (int)sdo.subindex <<" with value " << (int)value << ", read value " << response_sdo.value << std::endl;
                    return false;
                }
                // Restart timer
                time_start = std::chrono::high_resolution_clock::now();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return true;
    }

    void initDeviceManagerThread(std::string chainName, std::function<void (std::string)> const& deviceManager)
    {
        std::thread device_manager_thread(deviceManager, chainName);
        device_manager_thread.detach();
        //managerThreads.push_back(device_manager_thread);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void deviceManager(std::string chainName)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> time_start, time_end;
        std::chrono::duration<double> elapsed_seconds;
        time_start = std::chrono::high_resolution_clock::now();

        while (true)
        {
            time_end = std::chrono::high_resolution_clock::now();
            elapsed_seconds = time_end-time_start;

            for (auto id : canopen::deviceGroups[chainName].getCANids())
            {
                if(elapsed_seconds.count() > 2)
                {
                    time_start = std::chrono::high_resolution_clock::now();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    void RPDO2_outgoing(uint8_t CANid, int32_t target_position, uint32_t max_velocity)
    {
        TPCANMsg msg;
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = COB_PDO2_RX + CANid;
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

    void RPDO4_outgoing(uint8_t CANid, int16_t target_torque)
    {
        TPCANMsg msg;
        std::memset(&msg, 0, sizeof(msg));
        msg.ID = COB_PDO4_RX + CANid;
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

    void TPDO1_incoming_motors(uint8_t CANid, const TPCANRdMsg m)
    {

        uint16_t mydata_low = m.Msg.DATA[0];
        uint16_t mydata_high = m.Msg.DATA[1];

        devices[CANid].statusword = mydata_low + (mydata_high << 8);
        int8_t mode_display = m.Msg.DATA[2];

        uint16_t low_byte, high_byte;
        low_byte = m.Msg.DATA[3];
        high_byte = m.Msg.DATA[4];
        uint16_t limit_switch = low_byte + (high_byte << 8);

        low_byte = m.Msg.DATA[5];
        high_byte = m.Msg.DATA[6];
        uint16_t inputs = low_byte + (high_byte << 8);

        bool hardware_limit_positive = limit_switch & 0x02;
        bool hardware_limit_negative = limit_switch & 0x01;
        bool input3 = inputs & 0x04;
        bool input4 = inputs & 0x08;

        devices[CANid].setNegativeLimit(hardware_limit_negative);
        devices[CANid].setPositiveLimit(hardware_limit_positive);
        devices[CANid].setInput3(input3);
        devices[CANid].setInput4(input4);

        bool ready_switch_on = mydata_low & 0x01;
        bool switched_on = mydata_low & 0x02;
        bool op_enable = mydata_low & 0x04;
        bool fault = mydata_low & 0x08;
        bool volt_enable = mydata_low & 0x10;
        bool quick_stop = mydata_low & 0x20;
        bool switch_on_disabled = mydata_low & 0x40;
        bool warning = mydata_low & 0x80;

        bool mode_specific = mydata_high & 0x01;
        bool remote = mydata_high & 0x02;
        bool target_reached = mydata_high & 0x04;
        bool internal_limit = mydata_high & 0x08;
        bool op_specific = mydata_high & 0x10;
        bool op_specific1 = mydata_high & 0x20;
        bool man_specific1 = mydata_high & 0x40;
        bool man_specific2 = mydata_high & 0x80;
        bool ip_mode = ready_switch_on & switched_on & op_enable & volt_enable;

        if(!ready_switch_on)
        {
            if(fault)
            {
                devices[CANid].setMotorState(canopen::MS_FAULT);
            }
            else if(switch_on_disabled)
            {
                devices[CANid].setMotorState(canopen::MS_SWITCHED_ON_DISABLED);
            }
            else
                devices[CANid].setMotorState(canopen::MS_NOT_READY_TO_SWITCH_ON);
        }

        else
        {
            if(switched_on)
            {
                if(op_enable)
                {

                    //if(volt_enable)
                    // {
                    devices[CANid].setMotorState(canopen::MS_OPERATION_ENABLED);
                    // }

                }
                else
                    devices[CANid].setMotorState(canopen::MS_SWITCHED_ON);
            }
            else if(!quick_stop)
                devices[CANid].setMotorState(canopen::MS_QUICK_STOP_ACTIVE);

            else
                devices[CANid].setMotorState(canopen::MS_READY_TO_SWITCH_ON);

        }

        if(fault & op_enable & switched_on & ready_switch_on)
            devices[CANid].setMotorState(canopen::MS_FAULT_REACTION_ACTIVE);


        devices[CANid].setFault(fault);
        devices[CANid].setIPMode(ip_mode);
        devices[CANid].setHoming(op_specific);
        devices[CANid].setOpSpec0(op_specific);
        devices[CANid].setOpSpec1(op_specific1);
        devices[CANid].setManSpec1(man_specific1);
        devices[CANid].setManSpec2(man_specific2);
        devices[CANid].setInternalLimits(internal_limit);
        devices[CANid].setTargetReached(target_reached);
        devices[CANid].setRemote(remote);
        devices[CANid].setModeSpec(mode_specific);
        devices[CANid].setWarning(warning);
        devices[CANid].setSwitchOnDisable(switch_on_disabled);
        devices[CANid].setQuickStop(quick_stop);
        devices[CANid].setOpEnable(op_enable);
        devices[CANid].setVoltageEnabled(volt_enable);
        devices[CANid].setReadySwitchON(ready_switch_on);
        devices[CANid].setSwitchON(switched_on);

        devices[CANid].setCurrentModeofOperation(mode_display);
    }

    void TPDO2_incoming_motors(uint8_t CANid, const TPCANRdMsg m)
    {
        int32_t newPos = m.Msg.DATA[0] + (m.Msg.DATA[1] << 8) + (m.Msg.DATA[2] << 16) + (m.Msg.DATA[3] << 24);
        int16_t newVel = m.Msg.DATA[4] + (m.Msg.DATA[5] << 8);

        devices[CANid].setActualPos(newPos);
        devices[CANid].setActualVel(newVel);

        devices[CANid].setTimeStamp_sec(std::chrono::seconds(m.dwTime / 1000));  // convert milliseconds to seconds
        devices[CANid].setTimeStamp_nsec(std::chrono::nanoseconds(m.wUsec + (m.dwTime % 1000000) * 1000000));  // convert remaining milliseconds to nanoseconds
    }

    void TPDO1_incoming_io(uint8_t CANid, const TPCANRdMsg m)
    {
        devices[CANid].inputs = 0;
        for(int i=0; i<8; ++i)
        {
            devices[CANid].inputs += ((uint64_t)m.Msg.DATA[i]) << (8*i);
        }
    }

    void EMCY_incoming(uint8_t CANid, const TPCANRdMsg m)
    {
        std::stringstream error_stream;

        uint16_t error_code =  m.Msg.DATA[0] +  m.Msg.DATA[1] << 8;
        uint8_t error_class = m.Msg.DATA[2];
        uint8_t error_number = m.Msg.DATA[3];


        error_stream << std::hex << "ERROR: " << NanotecErrorNumber[error_number] << " ERROR CATEGORIES: ";

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

        std::cout << error_stream << std::endl;
        devices[CANid].last_error = error_stream.str();
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
        while(true)
        {
            //std::cout << "Reading incoming data" << std::endl;
            TPCANRdMsg m;
            errno = LINUX_CAN_Read(h, &m);
            if(errno != 0)
                std::cout << "LINUX_CAN_Read() " << errno << std::endl;
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
                SDOkey sdoKey(m);
                if(sdo_protect)
                {
                    std::copy(std::begin(m.Msg.DATA), std::end(m.Msg.DATA), std::begin(protect_msg));
                    sdo_protect = false;
                }
                else
                {
                    if (incomingManufacturerDetails.find(sdoKey) != incomingManufacturerDetails.end())
                        incomingManufacturerDetails[sdoKey](m.Msg.ID - COB_SDO_TX, m.Msg.DATA);
                    else
                        sdo_incoming(m.Msg.ID - COB_SDO_TX, m.Msg.DATA);
                }
            }

            // incoming NMT heartbeat
            else if (m.Msg.ID >= COB_NODEGUARD && m.Msg.ID < COB_MAX)
            {
                uint8_t CANid = m.Msg.ID - COB_NODEGUARD;

                std::cout << std::hex << "Received NMT message from CANid " << (int)CANid << ". Current state: " << nmt_state[m.Msg.DATA[0]] << std::endl;
                std::map<uint8_t,Device>::const_iterator search = devices.find(CANid);
                if(search != devices.end())
                {
                    // std::cout << "Found " << (u_int16_t)search->first << "\n";
                    devices[CANid].setNMTInit(true);
                }
                else
                {
                    std::cout << "Received bootup from node " << (int)CANid << " which I do not know what to do with...ignoring" << std::endl;
                }

            }

            else
            {
                std::cout << "Received unknown message" << std::endl;
            }
        }
    }

    void manufacturer_incoming(uint8_t CANid, BYTE data[8])
    {
        sdo_protect = true;

        if(data[1]+(data[2]<<8) == 0x1008)
        {
            std::vector<char> manufacturer_device_name = canopen::obtainManDevName(CANid, data[4]);

            devices[CANid].setManufacturerDevName(manufacturer_device_name);
        }
    }

    std::vector<uint8_t> obtainVendorID(uint8_t CANid)
    {
        canopen::uploadSDO(CANid, canopen::IDENTITYVENDORID);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::vector<uint16_t> obtainProdCode(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m)
    {
        canopen::uploadSDO(CANid, canopen::IDENTITYPRODUCTCODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::vector<uint16_t> product_code;

        canopen::processSingleSDO(CANid, m);

        uint8_t id4 = m->Msg.DATA[4];
        uint8_t id3 = m->Msg.DATA[5];
        uint8_t id2 = m->Msg.DATA[6];
        uint8_t id1 = m->Msg.DATA[7];

        product_code.push_back(id1);
        product_code.push_back(id2);
        product_code.push_back(id3);
        product_code.push_back(id4);

        return product_code;

    }

    uint16_t obtainRevNr(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m)
    {
        canopen::uploadSDO(CANid, canopen::IDENTITYREVNUMBER);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));


        canopen::processSingleSDO(CANid, m);

        uint8_t rev_number = m->Msg.DATA[4];

        return rev_number;

    }

    std::vector<char> obtainManDevName(uint8_t CANid, int size_name)
    {

        std::vector<char> manufacturer_device_name;

        canopen::requestDataBlock1(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        for (auto it : protect_msg)
        {
            if(manufacturer_device_name.size() <= size_name)
                manufacturer_device_name.push_back(it);
        }

        return manufacturer_device_name;

    }

    std::vector<char> obtainManHWVersion(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m)
    {
        canopen::uploadSDO(CANid, canopen::MANUFACTURERHWVERSION);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        std::vector<char> manufacturer_hw_version;

        canopen::processSingleSDO(CANid, m);

        int size = m->Msg.DATA[4];

        canopen::requestDataBlock1(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_hw_version.size() <= size)
                manufacturer_hw_version.push_back(it);
        }


        canopen::requestDataBlock2(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_hw_version.size() <= size)
                manufacturer_hw_version.push_back(it);
        }

        return manufacturer_hw_version;
    }

    std::vector<char> obtainManSWVersion(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m)
    {
        std::vector<char> manufacturer_sw_version;

        canopen::uploadSDO(CANid, canopen::MANUFACTURERSOFTWAREVERSION);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);

        int size = (uint8_t)m->Msg.DATA[4];

        canopen::requestDataBlock1(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }


        canopen::requestDataBlock2(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }

        canopen::requestDataBlock1(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }

        canopen::requestDataBlock2(CANid);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        canopen::processSingleSDO(CANid, m);


        for (auto it : m->Msg.DATA)
        {
            if(manufacturer_sw_version.size() <= size)
                manufacturer_sw_version.push_back(it);
        }

        return manufacturer_sw_version;

    }

    void sdo_incoming(uint8_t CANid, BYTE data[8])
    {
        // data[0] -> command byte
        uint16_t sdo_id = data[1]+(data[2]<<8);
        uint8_t sdo_id_sub = data[3];
        response_sdo.can_id = CANid;
        response_sdo.index = sdo_id;
        response_sdo.subindex = sdo_id_sub;
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
            auto iter = sdo_abort_messages.find(abort_code);
            std::string error_message = "SDO Abort";
	     if ( iter != sdo_abort_messages.end())
            {
                error_message = (*iter).second;
            }
            std::cout << std::hex << "SDO abort from CAN id " << (int)CANid << " for SDO 0x" << sdo_id << "s" << (int)sdo_id_sub << " with the following error message:" << std::endl;
            std::cout << " " << error_message << std::endl;
        }
        else // no idea what I received
        {
            std::cout << std::hex << "Received SDO from 0x" << (int)CANid << " with id 0x" << sdo_id << "s" << (int)sdo_id_sub << " and command byte 0x" << (int)data[0] << "  DATA: " << (int)data[4] << " " << (int)data[5] << " " << (int)data[6] << " " << (int)data[7] << std::endl;
        }

        // check if SDO was requested
        if(sdo_id == requested_sdo.index && sdo_id_sub == requested_sdo.subindex && CANid == requested_sdo.can_id)
        {
            //std::cout << "requested sdo received " << response_sdo.value << std::endl;
            requested_sdo.confirmed = true;
            requested_sdo.value = response_sdo.value;
        }
    }

    void processSingleSDO(uint8_t CANid, std::shared_ptr<TPCANRdMsg> message)
    {
        message->Msg.ID = 0x00;

        while (message->Msg.ID != (COB_SDO_TX+CANid))
        {
            LINUX_CAN_Read(canopen::h, message.get());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void disableRPDO(uint8_t id, int object)
    {
        int32_t data;
        switch(object)
        {
            case 0:
                data = (canopen::RPDO1_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 1:
                data = (canopen::RPDO2_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 2:
                data = (canopen::RPDO3_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            case 3:
                data = (canopen::RPDO4_msg + id)  + (0x00 << 16) + (0x80 << 24);
                break;
            default:
                std::cout << "BAD OBJECT NUMBER IN disableRPDO! Number is " << object << std::endl;
                return;
        }
        sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
    }

    void clearRPDOMapping(uint8_t id, int object)
    {
        sendSDO(id, SDOkey(RPDO_map.index+object,0x00), (uint8_t)0x00);
    }

    void makeRPDOMapping(uint8_t id, int object, std::vector<std::string> registers, std::vector<int> sizes , u_int8_t sync_type)
    {
        int counter;
        for(counter=0; counter < registers.size();counter++)
        {
            int index_data;

            std::stringstream str_stream;
            str_stream << registers[counter];
            str_stream >> std::hex >> index_data;

            int32_t data = (sizes[counter]) + (index_data << 8);
            sendSDO(id, SDOkey(RPDO_map.index + object, counter + 1), data);
        }

        sendSDO(id, SDOkey(RPDO.index+object,0x02), u_int8_t(sync_type));
        std::cout << std::hex << "Mapping " << counter << " objects at CANid " << (int)id << " to RPDO" << object + 1 << std::endl;
        sendSDO(id, SDOkey(RPDO_map.index+object,0x00), uint8_t(counter));
    }

    void enableRPDO(uint8_t id, int object)
    {
        int32_t data;
        switch(object)
        {
            case 0:
                data = COB_PDO1_RX + id;
                break;
            case 1:
                data = COB_PDO2_RX + id;
                break;
            case 2:
                data = COB_PDO3_RX + id;
                break;
            case 3:
                data = COB_PDO4_RX + id;
                break;
            default:
                std::cout << "wrong object number in enableRPDO" << std::endl;
                return;
        }
        sendSDO(id, SDOkey(RPDO.index+object,0x01), data);
    }

    void disableTPDO(uint8_t id, int object)
    {
        int32_t data;
        switch(object)
        {
            case 0:
                data = (canopen::TPDO1_msg + id)  + (1 << 31);
                break;
            case 1:
                data = (canopen::TPDO2_msg + id)  + (1 << 31);
                break;
            case 2:
                data = (canopen::TPDO3_msg + id)  + (1 << 31);
                break;
            case 3:
                data = (canopen::TPDO4_msg + id)  + (1 << 31);
                break;
            default:
                std::cout << "Incorrect object for mapping" << std::endl;
                return;
        }
        sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
    }

    void clearTPDOMapping(uint8_t id, int object)
    {
        sendSDO(id, SDOkey(TPDO_map.index+object,0x00), uint8_t(0x00));
    }

    void makeTPDOMapping(uint8_t id, int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type)
    {
        int counter;
        for(counter = 0; counter < registers.size();counter++)
        {
            int index_data;

            std::stringstream str_stream;
            str_stream << registers[counter];
            str_stream >> std::hex >> index_data;

            int32_t data = (sizes[counter]) + (index_data << 8);
            sendSDO(id, SDOkey(TPDO_map.index + object, counter + 1), data);
        }

        sendSDO(id, SDOkey(TPDO.index+object,0x02), u_int8_t(sync_type));
        std::cout << std::hex << "Mapping " << counter << " objects at CANid " << (int)id << " to TPDO" << object + 1 << std::endl;
        sendSDO(id, SDOkey(TPDO_map.index+object,0x00), uint8_t(counter));
    }

    void enableTPDO(uint8_t id, int object)
    {
        int32_t data;
        switch(object)
        {
            case 0:
                data = COB_PDO1_TX + id;
                break;
            case 1:
                data = COB_PDO2_TX + id;
                break;
            case 2:
                data = COB_PDO3_TX + id;
                break;
            case 3:
                data = COB_PDO4_TX + id;
                break;
            default:
                std::cout << "Incorrect object number handed over to enableTPDO" << std::endl;
                return;
        }
        sendSDO(id, SDOkey(TPDO.index+object,0x01), data);
    }

}
