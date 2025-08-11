#ifndef xRouteDefs_h
#define xRouteDefs_h
#include <Arduino.h>
class MyController
{
public:
    enum DeviceType
    {
        XROUTE_PRO,
        XROUTE_MASTER
    };
    DeviceType deviceType;
    String Name;
    int relCount;
    int dimCount;
    int fltCount;
    int motorCount;
    MyController(DeviceType type)
    {
        switch (type)
        {
        case XROUTE_PRO:
            log_i("[xRouteDefs]: XROUTE_PRO");
            Name = "xRoute-Pro";
            relCount = 8;
            dimCount = 5;
            fltCount = 2;
            motorCount = 1;
            break;
        case XROUTE_MASTER:
            log_i("[xRouteDefs]: XROUTE_MASTER");
            Name = "xRoute-Master";
            relCount = 16;
            dimCount = 7;
            fltCount = 3;
            motorCount = 2;
            break;
        default:
            log_e("[xRouteDefs]: ERROR! Invalid device type");
            Name = "NO_NAME";
            relCount = 8;
            dimCount = 5;
            fltCount = 2;
            motorCount = 1;
            break;
        }
    }
};

#endif
