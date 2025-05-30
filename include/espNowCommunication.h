#ifndef ESP_NOW_COMMUNICATION_H
#define ESP_NOW_COMMUNICATION_H

#include <Arduino.h>
#include <esp_now.h>

class EspNowCommunication {
public:
    EspNowCommunication();
    bool init();
    bool sendMessage(const uint8_t *data, size_t len);
    void handleReceivedMessages();
    void setPeerMacAddress(const uint8_t *mac);

private:
    static EspNowCommunication *instance;  // Singleton instance

    uint8_t peerAddress[6];
    String receivedMessage;
    static void onReceive(const uint8_t *mac, const uint8_t *data, int len);
};

#endif // ESP_NOW_COMMUNICATION_H
