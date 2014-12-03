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

#ifndef CANOPEN_H
#define CANOPEN_H

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <cstring>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <cstdlib>
#include <thread>
#include <math.h>
#include <libpcan.h>
#include <utility>
#include <fcntl.h>    // for O_RDWR
#include <stdint.h>
#include <inttypes.h>
#include "schunkErrors.h"
#include "nanotecErrors.h"
#include <queue>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace canopen{
    extern std::string baudRate;
    extern bool canbus_error;
    extern HANDLE h;
    extern bool atFirstInit;
    extern std::vector<std::string> openDeviceFiles;

    /***************************************************************/
    // Define baudrates variables for accessing as string
    // this overrrides the definitions from the libpcan.h
    /**************************************************************/

    static std::map<std::string, uint32_t> baudrates = {
        {"1M" , CAN_BAUD_1M},
        {"500K" , CAN_BAUD_500K},
        {"250K" , CAN_BAUD_250K},
        {"125K" , CAN_BAUD_125K},
        {"100K" , CAN_BAUD_100K},
        {"50K" , CAN_BAUD_50K},
        {"20K" , CAN_BAUD_20K},
        {"10K" , CAN_BAUD_10K},
        {"5K" , CAN_BAUD_5K}
    };

    struct ProfilePosition
    {
        int32_t target;
        uint32_t max_velocity;
        u_int16_t control_word;
        uint32_t jerk;
    };

    /***************************************************************/
    //		    define classes and structs
    /***************************************************************/
    class ObjectKey{
    public:
        uint16_t index;
        uint8_t subindex;
        uint8_t size;

        ObjectKey(TPCANRdMsg m):
            index((m.Msg.DATA[2] << 8) + m.Msg.DATA[1]),
            subindex(m.Msg.DATA[3]),
            size(4) {}

        ObjectKey(uint16_t i, uint8_t s, uint8_t object_size):
            index(i),
            subindex(s),
            size(object_size) {}

        ObjectKey() {}
    };

    class SDOanswer
    {
    public:
        uint8_t can_id;
        ObjectKey object;
        int32_t value;
        bool aborted;
        bool confirmed;

        SDOanswer():
            can_id(0),
            confirmed(false){}
    };

    class JointState
    {
    public:
        double position;
        double velocity;
        ros::Time stamp;
        int32_t ticks_per_rad_or_meter;

        JointState():
            position(0.0),
            velocity(0.0),
            stamp(ros::Time::now()),
            ticks_per_rad_or_meter(4096)
        {}
    };

    void errorword_incoming(uint8_t CANid, BYTE data[8]);

    DWORD CAN_Write_debug(HANDLE h, TPCANMsg *msg);

    extern std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;

    /***************************************************************/
    //			define state machine functions
    /***************************************************************/

    void sendNMT(uint8_t CANid, uint8_t command);

    /***************************************************************/
    //	define init variables and functions
    /***************************************************************/

    bool openConnection(std::string devName, std::string baudrate);

    /***************************************************************/
    //	define NMT constants, variables and functions
    /***************************************************************/

    const uint8_t NMT_START_REMOTE_NODE = 0x01;
    const uint8_t NMT_STOP_REMOTE_NODE = 0x02;
    const uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
    const uint8_t NMT_RESET_NODE = 0x81;
    const uint8_t NMT_RESET_COMMUNICATION = 0x82;

    /***************************************************************/
    //		Error Constants for Error Register
    /***************************************************************/

    static unsigned char const EMC_k_1001_GENERIC        = 0x01;
    static unsigned char const EMC_k_1001_CURRENT        = 0x02;
    static unsigned char const EMC_k_1001_VOLTAGE        = 0x04;
    static unsigned char const EMC_k_1001_TEMPERATURE    = 0x08;
    static unsigned char const EMC_k_1001_COMMUNICATION  = 0x10;
    static unsigned char const EMC_k_1001_DEV_PROF_SPEC  = 0x20;
    static unsigned char const EMC_k_1001_RESERVED       = 0x40;
    static unsigned char const EMC_k_1001_MANUFACTURER   = 0x80;

    /***************************************************************/
    //		define motor state constants
    /***************************************************************/

    const std::string MS_NOT_READY_TO_SWITCH_ON = "NOT_READY_TO_SWITCH_ON";
    const std::string MS_FAULT = "FAULT";
    const std::string MS_SWITCHED_ON_DISABLED = "SWITCHED_ON_DISABLED";
    const std::string MS_READY_TO_SWITCH_ON = "READY_TO_SWITCH_ON";
    const std::string MS_SWITCHED_ON = "SWITCHED_ON";
    const std::string MS_OPERATION_ENABLED = "OPERATION_ENABLED";
    const std::string MS_QUICK_STOP_ACTIVE = "QUICK_STOP_ACTIVE";
    const std::string MS_FAULT_REACTION_ACTIVE = "FAULT_REACTION_ACTIVE";
    const std::string MS_START_UP = "START_UP";

    /***************************************************************/
    //		define SDO protocol constants and functions
    /***************************************************************/

    const ObjectKey STATUSWORD(0x6041, 0, 16);
    const ObjectKey ERRORWORD(0x1001, 0, 8);

    const ObjectKey CONTROLWORD(0x6040, 0, 16);
    const ObjectKey MODES_OF_OPERATION(0x6060, 0, 8);
    const ObjectKey MODES_OF_OPERATION_DISPLAY(0x6061, 0, 8);

    /* Constants for the PDO mapping */
    const int TPDO1_msg = 0x180;
    const int TPDO2_msg = 0x280;
    const int TPDO3_msg = 0x380;
    const int TPDO4_msg = 0x480;

    const int RPDO1_msg = 0x200;
    const int RPDO2_msg = 0x300;
    const int RPDO3_msg = 0x400;
    const int RPDO4_msg = 0x500;

    //PDO PARAMETERS
    const ObjectKey RPDO(0x1400, 0x0, 8);
    const ObjectKey RPDO_map(0x1600, 0x0, 8);
    const ObjectKey TPDO(0x1800, 0x0, 8);
    const ObjectKey TPDO_map(0x1A00, 0x0, 8);

    const uint16_t CONTROLWORD_SHUTDOWN = 6;
    const uint16_t CONTROLWORD_QUICKSTOP = 2;
    const uint16_t CONTROLWORD_SWITCH_ON = 7;
    const uint16_t CONTROLWORD_ENABLE_OPERATION = 15;
    const uint16_t CONTROLWORD_ENABLE_MOVEMENT = 31;
    const uint16_t CONTROLWORD_DISABLE_OPERATION = 7;
    const uint16_t CONTROL_WORD_DISABLE_VOLTAGE = 0x7D;
    const uint16_t CONTROLWORD_FAULT_RESET_0 = 0x00;
    const uint16_t CONTROLWORD_FAULT_RESET_1 = 0x80;
    const uint16_t CONTROLWORD_HALT = 0x100;

    const int8_t MODES_OF_OPERATION_HOMING_MODE = 0x6;
    const int8_t MODES_OF_OPERATION_PROFILE_POSITION_MODE = 0x1;
    const int8_t MODES_OF_OPERATION_VELOCITY_MODE = 0x2;
    const int8_t MODES_OF_OPERATION_PROFILE_VELOCITY_MODE = 0x3;
    const int8_t MODES_OF_OPERATION_PROFILE_TORQUE_MODE = 0x4;
    const int8_t MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE = 0x7;
    const int8_t MODES_OF_OPERATION_AUTO_SETUP = -2;
    const int8_t MODES_OF_OPERATION_AUTO_SETUP_OLD = -1;

    const u_int16_t COB_SYNC = 0x80;
    const u_int16_t COB_EMERGENCY = 0x80;
    const u_int16_t COB_TIME_STAMP = 0x100;
    const u_int16_t COB_PDO1_RX = 0x200;
    const u_int16_t COB_PDO2_RX = 0x300;
    const u_int16_t COB_PDO3_RX = 0x400;
    const u_int16_t COB_PDO4_RX = 0x500;
    const u_int16_t COB_PDO1_TX = 0x180;
    const u_int16_t COB_PDO2_TX = 0x280;
    const u_int16_t COB_PDO3_TX = 0x380;
    const u_int16_t COB_PDO4_TX = 0x480;
    const u_int16_t COB_SDO_TX = 0x580;
    const u_int16_t COB_SDO_RX = 0x600;
    const u_int16_t COB_NODEGUARD = 0x700;
    const u_int16_t COB_MAX = 0x800;

    const std::map<uint8_t, const std::string> nmt_state =
    {
        {0x00, "Bootup"},
        {0x04, "Stopped"},
        {0x05, "Operational"},
        {0x7F, "Pre-Operational"}
    };

    const std::map<uint32_t, const std::string> sdo_abort_messages =
    {
        {0x06040042, "The number and length of the objects to be mapped would exceed PDO length."},
        {0x05030000, "Toggle bit not alternated."},
        {0x05040000, "SDO protocol timed out."},
        {0x05040001, "Client/server command specifier not valid or unknown."},
        {0x05040002, "Invalid block size (block mode only)."},
        {0x05040003, "Invalid sequence number (block mode only)."},
        {0x05040004, "CRC error (block mode only)."},
        {0x05040005,"Out of memory."},
        {0x06010000,"Unsupported access to an object."},
        {0x06010001,"Attempt to read a write only object."},
        {0x06010002,"Attempt to write a read only object."},
        {0x06020000,"Object does not exist in the object dictionary."},
        {0x06040041,"Object cannot be mapped to the PDO."},
        {0x06040042,"The number and length of the objects to be mapped would exceed PDO length."},
        {0x06040043,"General parameter incompatibility reason."},
        {0x06040047,"General internal incompatibility in the device."},
        {0x06060000,"Access failed due to an hardware error."},
        {0x06070010,"Data type does not match, length of service parameter does not match"},
        {0x06070012,"Data type does not match, length of service parameter too high"},
        {0x06070013,"Data type does not match, length of service parameter too low"},
        {0x06090011,"Sub-index does not exist."},
        {0x06090030,"Invalid value for parameter (download only). "},
        {0x06090031,"Value of parameter written too high (download only)."},
        {0x06090032,"Value of parameter written too low (download only)."},
        {0x06090036,"Maximum value is less than minimum value."},
        {0x060A0023,"Resource not available: SDO connection"},
        {0x08000000,"General error"},
        {0x08000020,"Data cannot be transferred or stored to the application."},
        {0x08000021,"Data cannot be transferred or stored to the application because of local control."},
        {0x08000022,"Data cannot be transferred or stored to the application because of the present device state."},
        {0x08000023,"Object dictionary dynamic generation fails or no object dictionary is present (e.g. object dictionary is generated from file and generation fails because of an file error)."},
        {0x08000024,"No data available."}
    };

    const std::map<uint16_t, const std::string> error_codes =
    {
        {0x1000, "Generic Error"},
        {0x2310, "Output Current too high"},
        {0x3210, "Over/Undervoltage"},
        {0x4200, "Temperature Error"},
        {0x5010, "Sensorfehler oder X-Winkelwert außerhalb des Messbereiches"},
        {0x5020, "Sensorfehler oder Y-Winkelwert außerhalb des Messbereiches"},
        {0x7305, "Incremental Encoder 1 broken"},
        {0x8000, "CAN supervision error"},
        {0x8100, "Message Lost"},
        {0x8110, "CAN overrun, message lost"},
        {0x8120, "CAN in Error passive mode"},
        {0x8130, "Lifeguard or Heartbeat error"},
        {0x8140, "Recover from bus off"},
        {0x8210, "PDO not processed (length error)"},
        {0x8220, "PDO length exceeded"},
        {0x8611, "Following Error too high"},
        {0x8612, "Position Limit exceeded"}
    };

    const std::map<int8_t, const std::string> modesDisplay =
    {
        {-2, "Auto-Setup"},
        {0, "NO_MODE"},
        {1, "PROFILE_POSITION_MODE"},
        {2, "VELOCITY"},
        {3, "PROFILE_VELOCITY_MODE"},
        {4, "TORQUE_PROFILED_MODE"},
        {5, "MODE_5"},
        {6, "HOMING_MODE"},
        {7, "INTERPOLATED_POSITION_MODE"},
        {8, "CYCLIC_SYNCHRONOUS_POSITION"}
    };

    const u_int8_t SYNC_TYPE_ACYCLIC = 0x00;
    const u_int8_t SYNC_TYPE_CYCLIC = 0x01;
    const u_int8_t SYNC_TYPE_MANUFACTURER_SPECIFIC = 0xFE;
    const u_int8_t SYNC_TYPE_ASYNCHRONOUS = 0xFF;

    void EMCY_incoming(uint8_t CANid, const TPCANRdMsg m);

    /***************************************************************/
    //		define functions for receiving data
    /***************************************************************/

    void initListenerThread(std::function<void ()> const& listener);
    void defaultListener();
    void set_error_handler(void (*set_me)(const std::string&));
}

#endif
