#ifndef DEVICE_H
#define DEVICE_H

#include <ipa_canopen_core/canopen.h>
#include <boost/shared_ptr.hpp>
#include <canopen_interface/IOMessageTrigger.h>

using namespace canopen;

class Device
{
public:
    float polarity;
    uint64_t inputs;
    std::string device_type;
    std::string last_error;
    bool nmt_init;
    bool initialized;
    std::string name;
    SDOanswer requested_sdo;
    SDOanswer response_sdo;
    canopen_interface::IOMessageTrigger input_trigger;

    bool init(std::string deviceFile);
    virtual void init_pdo() = 0; // this function must be overwritten by child
    void uploadSDO(ObjectKey sdo);
    bool sendSDO(ObjectKey sdo, int value, bool verify = true, int32_t trials = 5, double timeout = 1.0);
    void sdo_incoming(BYTE data[8]);
    void nmt_incoming(BYTE data[8]);

    std::string getNMTState(){
        return NMTState_;
    }

    std::string getDeviceFile(){
        return deviceFile_;
    }

    uint8_t getCANid()
    {
        return CANid_;
    }

    std::string get_name()
    {
        return name;
    }

    void setNMTState(std::string nextState){
        NMTState_ = nextState;
    }

protected:
    std::vector<ObjectKey> tpdo_registers_, rpdo_registers_;
    void pdo_map(int pdo_id, uint8_t tsync_type, uint8_t rsync_type);
    uint8_t CANid_;
    uint64_t outputs_;

    Device(uint8_t CANid, std::string name, std::string type):
        CANid_(CANid),
        initialized(false),
        NMTState_("START_UP"),
        device_type(type),
        inputs(0), outputs_(0),
        polarity(1.0),
        nmt_init(false),
        name(name) {}

private:
    void makeRPDOMapping(int object, uint8_t sync_type);
    void disableRPDO(int object);
    void clearRPDOMapping(int object);
    void enableRPDO(int object);

    void makeTPDOMapping(int object, uint8_t sync_type);
    void disableTPDO(int object);
    void clearTPDOMapping(int object);
    void enableTPDO(int object);

    // test all possible int types allowed to make sure they are built into the library
    void test_sdo_types();

    double conversion_factor_;
    double offset_;
    std::string NMTState_;
    std::string deviceFile_;
};

typedef boost::shared_ptr<Device> DevicePtr;

extern std::map<uint8_t, DevicePtr> device_id_map;

#endif
