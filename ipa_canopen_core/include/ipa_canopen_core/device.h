#ifndef DEVICE_H
#define DEVICE_H

#include <ipa_canopen_core/canopen.h>
#include <boost/shared_ptr.hpp>
#include <canopen_interface/IOMessageTrigger.h>

using namespace canopen;

class Device
{
public:
    bool init(std::string deviceFile);
    void uploadSDO(ObjectKey sdo);
    bool sendSDO(ObjectKey sdo, int value, bool verify = true, int32_t trials = 5, double timeout = 1.0);
    void sdo_incoming(BYTE data[8]);
    void nmt_incoming(BYTE data[8]);
    void sendNMT(uint8_t command, bool send_to_all = false);
    uint8_t getCANid()
    {
        return CANid_;
    }
    std::string get_name()
    {
        return name;
    }

    uint64_t inputs;
    std::string device_type;
    bool nmt_init;
    bool initialized;
    std::string name;
    SDOanswer requested_sdo;
    SDOanswer response_sdo;
    canopen_interface::IOMessageTrigger input_trigger;

protected:
    Device(uint8_t CANid, std::string name, std::string type);
    void pdo_map(int pdo_id, uint8_t tsync_type, uint8_t rsync_type);
    bool set_sdos(ObjectKey sdo, std::string param);
    virtual void init_pdo() = 0; // this function must fill tpdo and rpdo registers and call pdo_map
    virtual void set_objects() = 0; // this function must set all sdo objects required
    std::vector<ObjectKey> tpdo_registers_, rpdo_registers_;
    uint8_t CANid_;
    uint64_t outputs_;

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
    std::string deviceFile_;
};

typedef boost::shared_ptr<Device> DevicePtr;

extern std::map<uint8_t, DevicePtr> device_id_map;

static const std::map<uint8_t, const std::string> nmt_state =
{
    {0x00, "Bootup"},
    {0x04, "Stopped"},
    {0x05, "Operational"},
    {0x7F, "Pre-Operational"}
};

static const std::map<uint32_t, const std::string> sdo_abort_messages =
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

static const int TPDO1_msg = 0x180;
static const int TPDO2_msg = 0x280;
static const int TPDO3_msg = 0x380;
static const int TPDO4_msg = 0x480;

static const int RPDO1_msg = 0x200;
static const int RPDO2_msg = 0x300;
static const int RPDO3_msg = 0x400;
static const int RPDO4_msg = 0x500;

static const int RPDO     = 0x1400;
static const int RPDO_map = 0x1600;
static const int TPDO     = 0x1800;
static const int TPDO_map = 0x1A00;

static const uint8_t SYNC_TYPE_ACYCLIC = 0x00;
static const uint8_t SYNC_TYPE_CYCLIC = 0x01;
static const uint8_t SYNC_TYPE_MANUFACTURER_SPECIFIC = 0xFE;
static const uint8_t SYNC_TYPE_ASYNCHRONOUS = 0xFF;

#endif
