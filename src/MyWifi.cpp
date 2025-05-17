#include<MyWifi.h>

MyWifi::MyWifi()
{    
}

MyWifi::~MyWifi()
{
}

bool MyWifi::isConnected()
{
    return WiFi.status() == WL_CONNECTED;
}

bool MyWifi::connectToWiFi()
{
    if (loadCredentials())
    {
        return WiFi.begin(_ssid.c_str(), _password.c_str()) == WL_CONNECTED;
    }
    return false;
}

bool MyWifi::saveCredentials(String ssid, String password)
{
    _ssid = ssid;
    _password = password;
    DynamicJsonDocument doc(1024);
    doc["ssid"] = ssid;
    doc["password"] = password;
    File file = SPIFFS.open(credential_path, "w");
    if (!file)
    {
        return false;
    }
    serializeJson(doc, file);
    file.close();
    return true;
}

bool MyWifi::loadCredentials()
{
    File file = SPIFFS.open(credential_path, "r");
    if (!file)
    {
        return false;
    }
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (error)
    {
        return false;
    }
    _ssid = doc["ssid"].as<String>();
    _password = doc["password"].as<String>();
    return true;
}   

void MyWifi::disconnect()
{
    WiFi.disconnect();
}

void MyWifi::setAutoConnect(bool enable)
{
    autoConnect = enable;
    WiFi.setAutoConnect(enable);
}

bool MyWifi::getAutoConnect()
{
    return autoConnect;
}

WiFiClient &MyWifi::getClient()
{
    return client;
}

String MyWifi::getSSID()
{
    return _ssid;
}

String MyWifi::getPassword()
{
    return _password;    
}   

String MyWifi::getLocalIP()
{
    return WiFi.localIP().toString();
}
