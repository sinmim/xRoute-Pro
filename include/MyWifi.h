#ifndef MYWIFI_H
#define MYWIFI_H

#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>


class MyWifi
{
private:
    // path for cridential being save
    const char *credential_path = "/wifi_credentials.json";
    WiFiClient client;
    String _ssid;
    String _password;
    bool autoConnect;
public:
    // sugest prototypes
    MyWifi();
    ~MyWifi();
    bool isConnected();
    bool connectToWiFi();
    bool saveCredentials(String ssid,String password);
    bool loadCredentials();
    void disconnect();
    void setAutoConnect(bool enable);
    bool getAutoConnect();
    WiFiClient &getClient();
    String getSSID();
    String getPassword();
    String getLocalIP();
};

#endif
