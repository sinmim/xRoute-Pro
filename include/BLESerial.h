#ifndef BLESerial_H
#define BLESerial_h
struct bleData
{
    bool deviceConnected = false;
    bool oldDeviceConnected = false;
    bool RxDataReadyFlag = false;
    bool TxDataSent = true;
    uint8_t txValue = 0;
    char bleRxStr[128];
    char bleTxStr[128];
};
void bleSetPass(uint32_t pass);
void setupBLE();
void BLEloop();
void BLEsend(char *str);
void remove_all_bonded_devices(void);
void bleSendLongString(String longString);

void bleDirectReadingStart();
void bleDirectReadingStop();
std::string bleDirectRead();

#endif
