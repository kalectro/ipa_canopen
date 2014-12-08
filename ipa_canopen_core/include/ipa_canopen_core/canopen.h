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
    extern boost::shared_ptr<ros::NodeHandle> n_p;
    extern std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;

    static std::map<std::string, uint32_t> baudrates = {
        {"1M" , CAN_BAUD_1M},
        {"500K" , CAN_BAUD_500K},
        {"250K" , CAN_BAUD_250K},
        {"125K" , CAN_BAUD_125K},
        {"100K" , CAN_BAUD_100K},
        {"50K" , CAN_BAUD_50K},
        {"20K" , CAN_BAUD_20K},
        {"10K" , CAN_BAUD_10K},
        {"5K" , CAN_BAUD_5K} };

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

    DWORD CAN_Write_debug(HANDLE h, TPCANMsg *msg);

    bool openConnection(std::string devName, std::string baudrate);
    void default_EMCY_incoming(uint8_t CANid, const TPCANRdMsg m);
    void initListenerThread(std::function<void ()> const& listener);
    void defaultListener();

    // nmt state constants
    static const uint8_t NMT_START_REMOTE_NODE = 0x01;
    static const uint8_t NMT_STOP_REMOTE_NODE = 0x02;
    static const uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
    static const uint8_t NMT_RESET_NODE = 0x81;
    static const uint8_t NMT_RESET_COMMUNICATION = 0x82;

    // error code constants
    static const unsigned char EMC_k_1001_GENERIC        = 0x01;
    static const unsigned char EMC_k_1001_CURRENT        = 0x02;
    static const unsigned char EMC_k_1001_VOLTAGE        = 0x04;
    static const unsigned char EMC_k_1001_TEMPERATURE    = 0x08;
    static const unsigned char EMC_k_1001_COMMUNICATION  = 0x10;
    static const unsigned char EMC_k_1001_DEV_PROF_SPEC  = 0x20;
    static const unsigned char EMC_k_1001_RESERVED       = 0x40;
    static const unsigned char EMC_k_1001_MANUFACTURER   = 0x80;

    // motor state constants
    static const std::string MS_NOT_READY_TO_SWITCH_ON = "NOT_READY_TO_SWITCH_ON";
    static const std::string MS_FAULT = "FAULT";
    static const std::string MS_SWITCHED_ON_DISABLED = "SWITCHED_ON_DISABLED";
    static const std::string MS_READY_TO_SWITCH_ON = "READY_TO_SWITCH_ON";
    static const std::string MS_SWITCHED_ON = "SWITCHED_ON";
    static const std::string MS_OPERATION_ENABLED = "OPERATION_ENABLED";
    static const std::string MS_QUICK_STOP_ACTIVE = "QUICK_STOP_ACTIVE";
    static const std::string MS_FAULT_REACTION_ACTIVE = "FAULT_REACTION_ACTIVE";
    static const std::string MS_START_UP = "START_UP";

    static const int8_t MODES_OF_OPERATION_HOMING_MODE = 0x6;
    static const int8_t MODES_OF_OPERATION_PROFILE_POSITION_MODE = 0x1;
    static const int8_t MODES_OF_OPERATION_VELOCITY_MODE = 0x2;
    static const int8_t MODES_OF_OPERATION_PROFILE_VELOCITY_MODE = 0x3;
    static const int8_t MODES_OF_OPERATION_PROFILE_TORQUE_MODE = 0x4;
    static const int8_t MODES_OF_OPERATION_INTERPOLATED_POSITION_MODE = 0x7;
    static const int8_t MODES_OF_OPERATION_AUTO_SETUP = -2;
    static const int8_t MODES_OF_OPERATION_AUTO_SETUP_OLD = -1;

    static const uint16_t COB_SYNC = 0x80;
    static const uint16_t COB_EMERGENCY = 0x80;
    static const uint16_t COB_TIME_STAMP = 0x100;
    static const uint16_t COB_PDO1_RX = 0x200;
    static const uint16_t COB_PDO2_RX = 0x300;
    static const uint16_t COB_PDO3_RX = 0x400;
    static const uint16_t COB_PDO4_RX = 0x500;
    static const uint16_t COB_PDO1_TX = 0x180;
    static const uint16_t COB_PDO2_TX = 0x280;
    static const uint16_t COB_PDO3_TX = 0x380;
    static const uint16_t COB_PDO4_TX = 0x480;
    static const uint16_t COB_SDO_TX = 0x580;
    static const uint16_t COB_SDO_RX = 0x600;
    static const uint16_t COB_NODEGUARD = 0x700;
    static const uint16_t COB_MAX = 0x800;

    static const std::map<int8_t, const std::string> modesDisplay =
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
        {8, "CYCLIC_SYNCHRONOUS_POSITION"} };

}

#endif
