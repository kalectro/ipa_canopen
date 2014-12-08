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
#include <ipa_canopen_core/device.h>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <ros/ros.h>


std::map<uint8_t, DevicePtr> device_id_map;
std::map<std::string, DevicePtr> device_name_map;

namespace canopen
{
    /***************************************************************/
    //			define global variables and functions
    /***************************************************************/
    std::string baudRate;
    bool canbus_error = false;
    HANDLE h;
    std::vector<std::string> openDeviceFiles;
    bool atFirstInit=true;
    boost::shared_ptr<ros::NodeHandle> n_p;

    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;
    std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingEMCYHandlers;

    std::string operation_mode_param;

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
        static int nreads, nwrites;
        static int counter;
        counter = 0;

        status = LINUX_CAN_Extended_Status(h, &nreads, &nwrites);
        if(nwrites > 31)
        {
            ROS_WARN_STREAM("Write queue size " << nwrites <<". Waiting for message " << std::hex << (int)msg->ID << " Data 0x" << (int)msg->DATA[0] << " 0x" << (int)msg->DATA[1] << " 0x" << (int)msg->DATA[2] << " 0x" << (int)msg->DATA[3] << " 0x" << (int)msg->DATA[4] << " 0x" << (int)msg->DATA[5] << " 0x" << (int)msg->DATA[6] << " 0x" << (int)msg->DATA[7]);
            ROS_WARN_STREAM("Waiting 500ms for recovery");
            ros::Duration(0.5).sleep();
        }
        else if(nwrites > 30)
        {
            ROS_WARN_STREAM("Write queue size " << nwrites <<". Waiting for message " << std::hex << (int)msg->ID << " Data 0x" << (int)msg->DATA[0] << " 0x" << (int)msg->DATA[1] << " 0x" << (int)msg->DATA[2] << " 0x" << (int)msg->DATA[3] << " 0x" << (int)msg->DATA[4] << " 0x" << (int)msg->DATA[5] << " 0x" << (int)msg->DATA[6] << " 0x" << (int)msg->DATA[7]);
            ROS_WARN_STREAM("Waiting 50ms for recovery");
            ros::Duration(0.05).sleep();
        }

        while(status & 0x80)
        {
            ROS_ERROR_STREAM("STATUS UGLY... we have " << nwrites << " writes waiting... no recovery possible");
            canbus_error = true;
            return -1;
        }
        DWORD return_me = LINUX_CAN_Write_Timeout(h, msg, 10000);
        if(return_me != 0)
            ROS_ERROR_STREAM("WRITE TIMEOUT ERROR! Code " << return_me);
        return return_me;
    }

    void default_EMCY_incoming(uint8_t CANid, const TPCANRdMsg m)
    {
        std::stringstream error_stream;
        uint16_t error_code =  m.Msg.DATA[0] +  (m.Msg.DATA[1] << 8);
        uint8_t error_class = m.Msg.DATA[2];
        uint8_t error_number = m.Msg.DATA[3];


        error_stream << std::hex << "EMCY from CANid " << (int)CANid << "! Error number: " << (int)error_number << " categories: ";

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

        error_stream << " code: " << std::hex << error_code;

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
            TPCANRdMsg m;
            errno = LINUX_CAN_Read(h, &m);
            // ROS_INFO_STREAM("Data incoming with message id " << std::hex << m.Msg.ID);
            if(errno != 0)
                ROS_ERROR_STREAM("LINUX_CAN_Read() error no " << errno);

            // incoming SYNC
            else if (m.Msg.ID == COB_SYNC)
            {
            }

            // incoming EMCY
            else if (m.Msg.ID >= COB_EMERGENCY && m.Msg.ID < COB_TIME_STAMP)
            {
                if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                    incomingPDOHandlers[m.Msg.ID](m);
                else
                    default_EMCY_incoming(m.Msg.ID - COB_EMERGENCY, m);
            }

            // incoming TIME
            else if (m.Msg.ID == COB_TIME_STAMP)
            {
            }

            // incoming PD0
            else if (m.Msg.ID >= COB_PDO1_TX && m.Msg.ID < COB_PDO4_RX)
            {
                if (incomingPDOHandlers.find(m.Msg.ID) != incomingPDOHandlers.end())
                {
                    incomingPDOHandlers[m.Msg.ID](m);
                }
            }

            // incoming SD0
            else if (m.Msg.ID >= COB_SDO_TX && m.Msg.ID < COB_SDO_RX)
            {
                int id = m.Msg.ID - COB_SDO_TX;
                if(device_id_map.find(id) != device_id_map.end())
                {
                    device_id_map[id]->sdo_incoming(m.Msg.DATA);
                }
                else
                {
                    ROS_ERROR_STREAM("Received SDO message from node 0x" << std::hex << id << " which is not part of known devices");
                }
            }

            // incoming NMT heartbeat
            else if (m.Msg.ID >= COB_NODEGUARD && m.Msg.ID < COB_MAX)
            {
                int id = m.Msg.ID - COB_NODEGUARD;
                if(device_id_map.find(id) != device_id_map.end())
                {
                    device_id_map[id]->nmt_incoming(m.Msg.DATA);
                }
                else
                {
                    ROS_WARN_STREAM("Received bootup from node 0x" << std::hex << id << " which I do not know what to do with...ignoring");
                }
            }
            else
            {
                ROS_ERROR_STREAM("Received unknown message with id 0x" << std::hex << m.Msg.ID << " and Data 0x" << (int)m.Msg.DATA[0] << " 0x" << (int)m.Msg.DATA[1] << " 0x" << (int)m.Msg.DATA[2] << " 0x" << (int)m.Msg.DATA[3] << " 0x" << (int)m.Msg.DATA[4] << " 0x" << (int)m.Msg.DATA[5] << " 0x" << (int)m.Msg.DATA[6] << " 0x" << (int)m.Msg.DATA[7] );
            }
        }
    }
}
