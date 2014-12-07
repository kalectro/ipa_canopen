#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ipa_canopen_core/device.h>

using namespace canopen;

class Motor : public Device
{
private:
    int8_t operation_mode_target_;
    std::string prefix_;
    void controlPDO();

public:
    std::string state;
    bool was_homed;
    std::string user_code;
    double max_jerk;
    std::deque <int16_t> analog0;
    std::deque <int16_t> analog1;
    double analog0_inhibit_time;
    double analog1_inhibit_time;
    bool has_encoder;
    bool use_analog;
    std::queue <ProfilePosition> position_commands;
    std::pair<bool, ros::Time> ack;
    int retry;
    int32_t nanoj_outputs;

    class Status
    {
    public:
        uint16_t statusword;
        uint16_t controlword;
        bool ready_switch_on;
        bool switched_on;
        bool op_enable;
        bool fault;
        bool volt_enable;
        bool quick_stop;
        bool switch_on_disabled;
        bool warning;
        bool target_reached;
        bool internal_limit;
        bool op_specific0;
        bool op_specific1;
        bool man_specific0;
        bool man_specific1;
        bool mode_specific;
        bool remote;
        int8_t actual_operation_mode;
        ros::Time stamp;
        std::string state;

        Status():
            statusword(0),
            controlword(0),
            ready_switch_on(false),
            switched_on(false),
            op_enable(false),
            fault(false),
            volt_enable(false),
            quick_stop(false),
            switch_on_disabled(false),
            warning(false),
            target_reached(false),
            internal_limit(false),
            op_specific0(false),
            op_specific1(false),
            man_specific0(false),
            man_specific1(false),
            mode_specific(false),
            remote(false),
            actual_operation_mode(0),
            stamp(ros::Time::now()),
            state("SWITCHED_ON_DISABLED")
        {}
    };

    JointState joint_state;
    Status status;

    Motor(uint8_t CANid, std::string name):
        Device(CANid, name, "motor"),
        state("SWITCHED_ON_DISABLED"),
        prefix_("motors/"),
        joint_state(),
        status(),
        use_analog(false),
        was_homed(false),
        user_code("none"),
        max_jerk(1000.0),
        operation_mode_target_(0),
        ack({true, ros::Time(0)}),
        retry(0),
        nanoj_outputs(0)
    {}

    void init_pdo();
    void set_objects();
    bool setOperationMode(int8_t targetMode, double timeout = 10.0);
    bool setMotorState(std::string targetState, double timeout = 10.0);
    void sendControlWord(uint16_t target_controlword);
    bool check_operation_mode(int8_t target_mode);
    void RPDO2_profile_position(int32_t target_position, uint32_t max_velocity);
    void RPDO4_position_rectified(int32_t profile_position);
    void RPDO4_jerk(uint32_t profile_jerk);
    void RPDO4_torque(int16_t target_torque);
    void TPDO1_incoming(const TPCANRdMsg m);
    void TPDO2_incoming(const TPCANRdMsg m);
    void TPDO3_incoming(const TPCANRdMsg m);
    void TPDO4_incoming(const TPCANRdMsg m);
    void setOutputs(uint64_t target_outputs);
    void error_cb(const TPCANRdMsg m);
};

typedef boost::shared_ptr<Motor> MotorPtr;

MotorPtr as_motor(DevicePtr ptr);

static const ObjectKey STATUSWORD(0x6041, 0, 16);

static const uint16_t CONTROLWORD_SHUTDOWN = 6;
static const uint16_t CONTROLWORD_QUICKSTOP = 2;
static const uint16_t CONTROLWORD_SWITCH_ON = 7;
static const uint16_t CONTROLWORD_ENABLE_OPERATION = 15;
static const uint16_t CONTROLWORD_ENABLE_MOVEMENT = 31;
static const uint16_t CONTROLWORD_DISABLE_OPERATION = 7;
static const uint16_t CONTROL_WORD_DISABLE_VOLTAGE = 0x7D;
static const uint16_t CONTROLWORD_FAULT_RESET_0 = 0x00;
static const uint16_t CONTROLWORD_FAULT_RESET_1 = 0x80;
static const uint16_t CONTROLWORD_HALT = 0x100;

#endif
