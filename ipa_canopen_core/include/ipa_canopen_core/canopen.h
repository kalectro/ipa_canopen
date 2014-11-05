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

namespace canopen{
    extern std::string baudRate;
    extern bool canbus_error;

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

    class Device{

    private:

        uint8_t CANid_;
        double conversion_factor_;
        double offset_;
        std::string NMTState_;
        std::string deviceFile_;
        std::string name_;
        std::string group_;

        std::vector<char> manufacturer_sw_version_;
        std::vector<char> manufacturer_hw_version_;
        std::vector<char> manufacturer_device_name_;

        std::vector<uint16_t> vendor_id_;
        std::vector<uint16_t> product_code_;
        uint16_t revision_number_;

        bool initialized_;
        bool nmt_init_;
        bool driveReferenced_;
        int64_t actualPos_;		// unit = ticks
        int32_t actualVel_;		// unit = ticks/sec
        std::chrono::seconds timeStamp_sec_;
        std::chrono::nanoseconds timeStamp_nsec_;

        bool hardware_limit_positive_;
        bool hardware_limit_negative_;
        bool input3_, input4_;


        bool ready_switch_on_;
        bool switched_on_;
        bool op_enable_;
        bool fault_;
        bool volt_enable_;
        bool quick_stop_;
        bool switch_on_disabled_;
        bool warning_;

        bool mode_specific_;
        bool remote_;
        bool target_reached_;
        bool internal_limit_;
        bool op_specific_;
        bool op_specific1_;
        bool man_specific1_;
        bool man_specific2_;

        double temperature_;

    public:

        float polarity;

        std::queue <ProfilePosition> position_commands;
        int32_t ticks_per_rad_or_meter;
        uint64_t inputs;
        uint64_t outputs;
        int16_t roll, pitch;
        bool is_motor;
        bool has_encoder;
        bool is_io_module;
        bool is_imu;
        uint16_t statusword;
        std::string last_error;
        uint16_t controlword;
        int8_t operation_mode_target;
        int8_t actual_operation_mode;
        bool was_homed;
        std::string motor_state;
        
        Device() :
            CANid_(-1),
            actualVel_(0),
            actualPos_(0),
            initialized_(false),
            NMTState_("START_UP"),
            motor_state("SWITCHED_ON_DISABLED"),
            is_motor(false),
            is_io_module(false),
            is_imu(false),
            roll(0), pitch(0),
            inputs(0), outputs(0),
            operation_mode_target(0),
            actual_operation_mode(0),
            controlword(0),
            was_homed(false),
            nmt_init_(false)
        {
        }

        Device(uint8_t CANid):
            CANid_(CANid),
            actualVel_(0),
            actualPos_(0),
            initialized_(false),
            NMTState_("START_UP"),
            motor_state("SWITCHED_ON_DISABLED"),
            is_motor(false),
            is_io_module(false),
            is_imu(false),
            roll(0), pitch(0),
            inputs(0), outputs(0),
            operation_mode_target(0),
            actual_operation_mode(0),
            controlword(0),
            was_homed(false),
            nmt_init_(false) {};

        bool getNMTInit(){
            return nmt_init_;
        }

        std::string getNMTState(){
            return NMTState_;
        }

        std::vector<char> getManufacturerSWVersion(){
            return manufacturer_sw_version_;
        }

        std::vector<char> getManufacturerHWVersion(){
            return manufacturer_hw_version_;
        }

        std::vector<char> getManufacturerDevName(){
            return manufacturer_device_name_;
        }

        std::vector<uint16_t> getVendorID(){
            return vendor_id_;
        }

        std::vector<uint16_t> getProdCode(){
            return product_code_;
        }

        uint16_t getRevNumber(){
            return revision_number_;
        }

        uint8_t getCANid(){
            return CANid_;
        }

        double getConversionFactor(){
            return conversion_factor_;
        }

        std::string getDeviceFile(){
            return deviceFile_;
        }
        std::string getGroup(){
            return group_;
        }
        std::string getName(){
            return name_;
        }

        bool getInitialized(){
            return initialized_;
        }


        bool getVoltageEnabled(){
            return volt_enable_;
        }

        double getDriverTemperature(){
            return temperature_;
        }

        bool getReadySwitchOn(){
            return ready_switch_on_;
        }

        bool getSwitchOn(){
            return switched_on_;
        }

        bool getOpEnabled(){
            return op_enable_;
        }

        bool getQuickStop(){
            return quick_stop_;
        }

        bool getSwitchOnDisabled(){
            return switch_on_disabled_;
        }

        bool getWarning(){
            return warning_;
        }

        bool getModeSpecific(){
            return mode_specific_;
        }

        bool getRemote(){
            return remote_;
        }
        bool getTargetReached(){
            return target_reached_;
        }

        bool getInternalLimits(){
            return internal_limit_;
        }


        bool getOpSpec0(){
            return op_specific_;
        }

        bool getOpSpec1(){
            return op_specific1_;
        }

        bool getManSpec1(){
            return man_specific1_;
        }

        bool getmanSpec2(){
            return man_specific2_;
        }

        bool getNegativeLimit(){
            return hardware_limit_negative_;
        }

        bool getPositiveLimit(){
            return hardware_limit_positive_;
        }

        bool getInput3(){
            return input3_;
        }

        bool getInput4(){
            return input4_;
        }

        bool getFault(){
            return fault_;
        }

        bool getDriveReferenced(){
            return driveReferenced_;
        }
        double getActualPos(){
            return actualPos_;
        }

        double getActualPosScaled(){
            return (double)actualPos_ / (double)ticks_per_rad_or_meter * (double)polarity;
        }

        double getActualVel(){
            return actualVel_;
        }

        inline uint32_t getTimeStamp_sec(){
            return timeStamp_sec_.count();
        }

        inline uint32_t getTimeStamp_nsec(){
            return timeStamp_nsec_.count();
        }

        void setActualPos(int64_t pos){
            actualPos_ = pos;
        }

        void setConversionFactor(double conversion_factor){
            conversion_factor_ = conversion_factor;
        }

        void setActualVel(double vel){
            actualVel_ = vel;
        }

        void setManufacturerSWVersion(std::vector<char> ms_version){
            manufacturer_sw_version_ = ms_version;
        }

        void setManufacturerHWVersion(std::vector<char> mh_version){
            manufacturer_hw_version_ = mh_version;
        }

        void setManufacturerDevName(std::vector<char> dev_name){
            manufacturer_device_name_ = dev_name;
        }

        void setVendorID(std::vector<uint16_t> v_id){
            vendor_id_ = v_id;
        }

        void setProdCode(std::vector<uint16_t> prod_code){
            product_code_ = prod_code;
        }


        void setRevNum(uint16_t rev_num){
            revision_number_ = rev_num;
        }


        void setNMTState(std::string nextState){
            NMTState_ = nextState;
        }


        void setVoltageEnabled(bool voltage_enabled){
            volt_enable_ = voltage_enabled;
        }

        void setDriverTemperature(double temperature){
            temperature_ = temperature;
        }

        void setReadySwitchON(bool r_switch_on){
            ready_switch_on_ = r_switch_on;
        }

        void setSwitchON(bool switch_on){
            switched_on_ = switch_on;
        }

        void setOpEnable(bool op_enable){
            op_enable_ = op_enable;
        }

        void setQuickStop(bool quick_stop){
            quick_stop_ = quick_stop;
        }

        void setSwitchOnDisable(bool switch_disabled){
            switch_on_disabled_ = switch_disabled;
        }

        void setWarning(bool warning){
            warning_ = warning;
        }


        void setModeSpec(bool modespec){
            mode_specific_ = modespec;
        }


        void setRemote(bool remote){
            remote_ = remote;
        }

        void setManSpec1(bool manspec1){
            man_specific1_ = manspec1;
        }

        void setTargetReached(bool target_reached){
            target_reached_ = target_reached;
        }

        void setInternalLimits(bool internal_limits){
            internal_limit_ = internal_limits;
        }


        void setManSpec2(bool manspec2){
            man_specific2_ = manspec2;
        }

        void setOpSpec1(bool opspec1){
            op_specific1_ = opspec1;
        }

        void setOpSpec0(bool opspec0){
            op_specific_ = opspec0;
        }

        void setPositiveLimit(bool pos_limit){
            hardware_limit_positive_ = pos_limit;
        }

        void setNegativeLimit(bool neg_limit){
            hardware_limit_negative_ = neg_limit;
        }

        void setInput3(bool value){
            input3_ = value;
        }

        void setInput4(bool value){
            input4_ = value;
        }

        void setNMTInit(bool nmt_limit)
        {
            nmt_init_ = nmt_limit;
        }

        void setFault(bool fault){
            fault_ = fault;
        }

        void setInitialized(bool initialized){
            initialized_ = initialized;
        }

        void setTimeStamp_sec(std::chrono::seconds timeStamp){
            timeStamp_sec_ = timeStamp;
        }

        void setTimeStamp_nsec(std::chrono::nanoseconds timeStamp){
            timeStamp_nsec_ = timeStamp;
        }
    };

    extern std::map<uint8_t, Device> devices;

    class DeviceGroup{

    private:

        std::vector<uint8_t> CANids_;
        std::vector<std::string> names_;
        bool initialized_;
        bool atFirstInit_;

    public:

        DeviceGroup() {};

        DeviceGroup(std::vector<uint8_t> CANids):
            CANids_(CANids) {};

        DeviceGroup(std::vector<uint8_t> CANids, std::vector<std::string> names):
            CANids_(CANids),
            names_(names),
            initialized_(false),
            atFirstInit_(true) {};


        std::vector<uint8_t> getCANids(){
            return CANids_;
        }

        std::vector<std::string> getNames(){
            return names_;
        }

        void setInitialized(bool initialized){
            initialized_ = initialized;
        }

        bool getInitialized(){
            return initialized_;
        }

        bool getFirstInit()
        {
            return atFirstInit_;
        }

        void setFirstInit(bool initialized)
        {
            initialized_ = initialized;
        }


        std::vector<double> getActualPos() {
            std::vector<double> actualPos;
            for (uint8_t CANid : CANids_)
                actualPos.push_back(devices[CANid].getActualPos());
            return actualPos;
        }

        std::vector<double> getActualVel() {
            std::vector<double> actualVel;
            for (auto CANid : CANids_)
                actualVel.push_back(devices[CANid].getActualVel());
            return actualVel;
        }
    };

    struct SDOkey{
        uint16_t index;
        uint8_t subindex;

        inline SDOkey(TPCANRdMsg m):
            index((m.Msg.DATA[2] << 8) + m.Msg.DATA[1]),
            subindex(m.Msg.DATA[3]) {};

        inline SDOkey(uint16_t i, uint8_t s):
            index(i),
            subindex(s) {};
    };

    struct SDOanswer{
        uint8_t can_id;
        uint16_t index;
        uint8_t subindex;
        int32_t value;
        bool aborted;
        bool confirmed;
    };

    /***************************************************************/
    //		define global variables and functions
    /***************************************************************/

    inline bool operator<(const SDOkey &a, const SDOkey&b) {
        return a.index < b.index || (a.index == b.index && a.subindex < b.subindex);
    }

    inline int32_t rad2mdeg(double phi){
        return static_cast<int32_t>(round(phi/(2*M_PI)*360000.0));
    }

    inline double mdeg2rad(int32_t alpha){
        return static_cast<double>(static_cast<double>(alpha)/360000.0*2*M_PI);
    }

    void sdo_incoming(uint8_t CANid, BYTE data[8]);
    void errorword_incoming(uint8_t CANid, BYTE data[8]);
    void manufacturer_incoming(uint8_t CANid, BYTE data[8]);

    extern std::map<std::string, DeviceGroup> deviceGroups;	// DeviceGroup name -> DeviceGroup object
    extern std::vector<std::thread> managerThreads;
    extern HANDLE h;
    extern std::vector<std::string> openDeviceFiles;
    extern bool atFirstInit;
    extern std::map<uint16_t, std::function<void (const TPCANRdMsg m)> > incomingPDOHandlers;

    extern SDOanswer requested_sdo;
    extern SDOanswer response_sdo;
    /***************************************************************/
    //			define state machine functions
    /***************************************************************/

    void setNMTState(uint8_t CANid, std::string targetState);
    bool setMotorState(uint8_t CANid, std::string targetState, double timeout = 10.0);
    bool setOperationMode(uint8_t CANid, int8_t targetMode, double timeout = 10.0);
    void output_error(std::string incoming_error = "ERROR");

    void makeRPDOMapping(uint8_t id, int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type);
    void disableRPDO(uint8_t id, int object);
    void clearRPDOMapping(uint8_t id, int object);
    void enableRPDO(uint8_t id, int object);

    void makeTPDOMapping(uint8_t, int object, std::vector<std::string> registers, std::vector<int> sizes, u_int8_t sync_type);
    void disableTPDO(uint8_t id, int object);
    void clearTPDOMapping(uint8_t id, int object);
    void enableTPDO(uint8_t id, int object);

    std::vector<char> obtainManSWVersion(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m);
    std::vector<char> obtainManHWVersion(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m);
    std::vector<char> obtainManDevName(uint8_t CANid, int size_name);
    std::vector<uint8_t> obtainVendorID(uint8_t CANid);
    uint16_t obtainRevNr(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m);
    std::vector<uint16_t> obtainProdCode(uint8_t CANid, std::shared_ptr<TPCANRdMsg> m);
    void readManErrReg(uint8_t CANid);


    /***************************************************************/
    //	define init variables and functions
    /***************************************************************/

    extern bool sdo_protect;
    extern BYTE protect_msg[];

    extern bool no_position;

    extern std::string operation_mode_param;

    bool openConnection(std::string devName, std::string baudrate);

    bool init(std::string deviceFile, std::string chainName, uint8_t CANid, uint8_t max_pdo_channels);

    void pdo_map(std::string chain_name, int pdo_id,
                 std::vector<std::string> tpdo_registers, std::vector<int> tpdo_sizes, u_int8_t tsync_type,
                 std::vector<std::string> rpdo_registers, std::vector<int> rpdo_sizes, u_int8_t rsync_type);


    /***************************************************************/
    //	define NMT constants, variables and functions
    /***************************************************************/

    const uint8_t NMT_START_REMOTE_NODE = 0x01;
    const uint8_t NMT_STOP_REMOTE_NODE = 0x02;
    const uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80;
    const uint8_t NMT_RESET_NODE = 0x81;
    const uint8_t NMT_RESET_COMMUNICATION = 0x82;

    extern TPCANMsg NMTmsg;

    inline void sendNMT(uint8_t CANid, uint8_t command)
    {
        TPCANMsg NMTmsg;
        std::memset(&NMTmsg, 0, sizeof(NMTmsg));
        NMTmsg.ID = 0;
        NMTmsg.MSGTYPE = 0x00;
        NMTmsg.LEN = 2;

        //std::cout << "Sending NMT. CANid: " << (uint16_t)CANid << "\tcommand: " << (uint16_t)command << std::endl;
        NMTmsg.DATA[0] = command;
        NMTmsg.DATA[1] = CANid;
        CAN_Write(h, &NMTmsg);
    }

    /***************************************************************/
    //		define NMT error control constants
    /***************************************************************/

    const SDOkey HEARTBEAT(0x1017,0x0);

    const uint16_t HEARTBEAT_TIME = 1500;

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

    const SDOkey STATUSWORD(0x6041, 0x0);
    const SDOkey ERRORWORD(0x1001, 0x0);
    const SDOkey MANUFACTURERDEVICENAME(0x1008, 0x0);
    const SDOkey MANUFACTURERHWVERSION(0x1009, 0x0);
    const SDOkey MANUFACTURERSOFTWAREVERSION(0x100A, 0x0);

    const SDOkey IDENTITYVENDORID(0x1018, 0x01);
    const SDOkey IDENTITYPRODUCTCODE(0x1018, 0x02);
    const SDOkey IDENTITYREVNUMBER(0x1018, 0x03);

    /*************************
     * Specific for schunk hardware
     ************************/
    const SDOkey SCHUNKLINE(0x200b, 0x1);
    const SDOkey SCHUNKDETAIL(0x200b, 0x3);
    /****************************************
     */

    const SDOkey CONTROLWORD(0x6040, 0x0);
    const SDOkey MODES_OF_OPERATION(0x6060, 0x0);
    const SDOkey MODES_OF_OPERATION_DISPLAY(0x6061, 0x0);
    const SDOkey SYNC_TIMEOUT_FACTOR(0x200e, 0x0);
    const SDOkey IP_TIME_UNITS(0x60C2, 0x1);
    const SDOkey IP_TIME_INDEX(0x60C2, 0x2);
    const SDOkey ERROR_CODE(0x603F, 0x0);
    const SDOkey ABORT_CONNECTION(0x6007, 0x0);
    const SDOkey QUICK_STOP(0x605A, 0x0);
    const SDOkey SHUTDOWN(0x605B, 0x0);
    const SDOkey DISABLE_CODE(0x605C, 0x0);
    const SDOkey HALT(0x605D, 0x0);
    const SDOkey FAULT(0x605E, 0x0);
    const SDOkey MODES(0x6060, 0x0);

    /* Constants for the PDO mapping */
    const int TPDO1_msg = 0x180;
    const int TPDO2_msg = 0x280;
    const int TPDO3_msg = 0x380;
    const int TPDO4_msg = 0x480;

    const int RPDO1_msg = 0x200;
    const int RPDO2_msg = 0x300;
    const int RPDO3_msg = 0x400;
    const int RPDO4_msg = 0x500;

    //TPDO PARAMETERS
    const SDOkey TPDO(0x1800, 0x0);

    //RPDO PARAMETERS
    const SDOkey RPDO(0x1400, 0x0);

    //TPDO MAPPING
    const SDOkey TPDO_map(0x1A00, 0x0);

    //RPDO MAPPING
    const SDOkey RPDO_map(0x1600, 0x0);

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

    const int8_t IP_TIME_INDEX_MILLISECONDS = 0xFD;
    const int8_t IP_TIME_INDEX_HUNDREDMICROSECONDS = 0xFC;
    const uint8_t SYNC_TIMEOUT_FACTOR_DISABLE_TIMEOUT = 0;

    const u_int8_t SYNC_TYPE_ACYCLIC = 0x00;
    const u_int8_t SYNC_TYPE_CYCLIC = 0x01;
    const u_int8_t SYNC_TYPE_MANUFACTURER_SPECIFIC = 0xFE;
    const u_int8_t SYNC_TYPE_ASYNCHRONOUS = 0xFF;

    void uploadSDO(uint8_t CANid, SDOkey sdo);
    void controlPDO(uint8_t CANid);
    void sendControlWord(uint8_t CANid, uint16_t target_controlword);
    void processSingleSDO(uint8_t CANid, std::shared_ptr<TPCANRdMsg> message);
    void requestDataBlock1(uint8_t CANid);
    void requestDataBlock2(uint8_t CANid);

    template< class IntType >
    bool sendSDO(uint8_t CANid, SDOkey sdo, IntType value, bool verify = true, int32_t trials = 5, double timeout = 1.0);
    // test all possible int types allowed to make sure they are built into the library
    void test_sdo_types();


    /***************************************************************/
    //		define PDO protocol functions
    /***************************************************************/

    void initDeviceManagerThread(std::string chainName, std::function<void (std::string)> const& deviceManager);
    void deviceManager(std::string chainName);

    void RPDO2_outgoing(uint8_t CANid, int32_t target_position, uint32_t max_velocity);
    void RPDO4_torque(uint8_t CANid, int16_t target_torque);
    void RPDO4_jerk(uint8_t CANid, uint32_t profile_jerk);
    void RPDO4_position_rectified(uint8_t CANid, int32_t profile_position);
    // void TPDO1_incoming_motors(uint8_t CANid, const TPCANRdMsg m);
    void TPDO2_incoming_motors(uint8_t CANid, const TPCANRdMsg m);
    void TPDO1_incoming_io(uint8_t CANid, const TPCANRdMsg m);
    void TPDO1_incoming_imu(uint8_t CANid, const TPCANRdMsg m);
    void EMCY_incoming(uint8_t CANid, const TPCANRdMsg m);

    /***************************************************************/
    //		define functions for receiving data
    /***************************************************************/

    void initListenerThread(std::function<void ()> const& listener);
    void defaultListener();
    void set_error_handler(void (*set_me)(const std::string&));
}

#endif
