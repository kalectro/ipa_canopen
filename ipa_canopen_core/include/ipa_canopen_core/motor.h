#ifndef MOTOR_H
#define MOTOR_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <ipa_canopen_core/device.h>
#include <sensor_msgs/JointState.h>

using namespace canopen;

class Motor : public Device
{
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

    struct ProfilePosition
    {
        double position;
        double velocity;
        double acceleration;
        double jerk;
        uint16_t control_word;
    };

    sensor_msgs::JointState joint_state;
    Status status;
    std::queue <ProfilePosition> position_commands;

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
        nanoj_outputs(0),
        ticks_per_rad_or_meter_(4096),
        polarity_(1.0)
    {
        joint_state.name.push_back(name);
        joint_state.position.push_back(0.0);
        joint_state.velocity.push_back(0.0);
    }

    void init_pdo();
    void set_objects();
    bool setOperationMode(int8_t targetMode, double timeout = 10.0);
    bool setMotorState(std::string targetState, double timeout = 10.0);
    void sendControlWord(uint16_t target_controlword);
    bool check_operation_mode(int8_t target_mode);
    void RPDO2_profile_position(double target_position, double velocity_limit);
    void RPDO4_position_rectified(double profile_position);
    void RPDO4_jerk(double profile_jerk);
    void RPDO4_torque(int16_t target_torque);
    void TPDO1_incoming(const TPCANRdMsg m);
    void TPDO2_incoming(const TPCANRdMsg m);
    void TPDO3_incoming(const TPCANRdMsg m);
    void TPDO4_incoming(const TPCANRdMsg m);
    void setOutputs(uint64_t target_outputs);
    void error_cb(const TPCANRdMsg m);
    double from_ticks_to_si(int32_t ticks);
    int from_si_to_ticks(double si);

private:
    int8_t operation_mode_target_;
    std::string prefix_;
    void controlPDO();
    double ticks_per_rad_or_meter_;
    int encoder_resolution_;
    float polarity_;
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

static const std::vector<std::string> nanotec_error_numbers =
{
    "No Error",
    "Input Voltage too high",
    "Output Current too high",
    "Input Voltage too low",
    "Can Bus Error",
    "Motor turns even though it is blocked",
    "NMT Master Nodeguarding Timeout",
    "Encoder Defekt",
    "Index Tick from Encoder not found during Auto Setup",
    "Encoder Error in A/B Track",
    "Positive Limit Switch activated and tolerance exceeded",
    "Negative Limit Switch activated and tolerance exceeded",
    "Temperature above 80 degrees",
    "Severe Following Error",
    "Flash is full",
    "Motor blocked",
    "Flash damaged",
    "PDO send timeout",
    "Hall sensor error"
};

static const std::map<uint16_t, const std::string> nanotec_error_codes =
{
    {0x1000, "Generic Error"},
    {0x2300, "Output Current too high"},
    {0x2310, "Output Current too high"},
    {0x3100, "Over/Undervoltage"},
    {0x3210, "Over/Undervoltage"},
    {0x4200, "Temperature Error"},
    {0x7212, "Motor blocked"},
    {0x7305, "Incremental Encoder 1 broken"},
    {0x7600, "Flash Storage full or corrupted"},
    {0x8000, "CAN supervision error"},
    {0x8100, "Message Lost"},
    {0x8110, "CAN overrun, message lost"},
    {0x8120, "CAN in Error passive mode"},
    {0x8130, "Lifeguard or Heartbeat error"},
    {0x8140, "Recover from bus off"},
    {0x8200, "Slave PDO timeout"},
    {0x8210, "PDO not processed (length error)"},
    {0x8220, "PDO length exceeded"},
    {0x8611, "Following Error too high"},
    {0x8612, "Position Limit exceeded"}
};

#endif
