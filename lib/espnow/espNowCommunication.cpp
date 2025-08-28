#include "espNowCommunication.h"

EspNowCommunication *EspNowCommunication::instance = nullptr;  // Initialize the singleton instance

EspNowCommunication::EspNowCommunication() : receivedMessage("") {
    // Initialize the singleton instance
    instance = this;
}

bool EspNowCommunication::init() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }

    // Set the callback function for receiving data
    esp_now_register_recv_cb([](const uint8_t *mac, const uint8_t *data, int len) {
        EspNowCommunication::instance->onReceive(mac, data, len);
    });

    return true;
}

bool EspNowCommunication::sendMessage(const uint8_t *data, size_t len) {
    esp_now_send(peerAddress, data, len);
    return true;
}

void EspNowCommunication::onReceive(const uint8_t *mac, const uint8_t *data, int len) {
    // Store the received message for later processing
    instance->receivedMessage.clear();
    for (int i = 0; i < len; i++) {
        instance->receivedMessage += (char)data[i];
        Serial.print("Received message: ");
    }
}

void EspNowCommunication::handleReceivedMessages() {
    // Check if there is a received message
    if (!receivedMessage.isEmpty()) {
        Serial.print("Received message: ");
        Serial.println(receivedMessage);

        // Process the received message as needed

        // Clear the received message buffer
        receivedMessage.clear();
    }
}

void EspNowCommunication::setPeerMacAddress(const uint8_t *mac) {
    memcpy(peerAddress, mac, 6);
}
