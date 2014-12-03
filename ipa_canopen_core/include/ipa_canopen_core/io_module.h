#ifndef IO_MODULE_H
#define IO_MODULE_H

#include <ipa_canopen_core/device.h>
#include <boost/shared_ptr.hpp>

using namespace canopen;

class IoModule : public Device
{
public:
    IoModule(uint8_t CANid, std::string name):
        Device(CANid, name, "io_module")
        {}

    void TPDO1_incoming(const TPCANRdMsg m);
    void init_pdo();
};

typedef boost::shared_ptr<IoModule> IoModulePtr;

IoModulePtr as_io_module(DevicePtr ptr);


#endif
