// userInfoKeys.h
#ifndef USERINFOKEYS_H
#define USERINFOKEYS_H

namespace NetworkKeys
{
    // Wi-Fi settings
    constexpr char WifiSSID[] = "ssid";
    constexpr char WifiPassword[] = "pass";
    // AP configuration
    constexpr char ApName[] = "apName";
    constexpr char ApPassword[] = "apPass";
    // hostName
    constexpr char HostName[] = "hostN";
    constexpr char Port[] = "port";
    // AP or STA
    constexpr char STA_AP[] = "STA_AP";
}

namespace jsonKeys
{
    constexpr char UI_CONFIG[] = "UI_CONFIG";
    constexpr char CONDITON_CONFIG[] = "CONDITION_CONFIG";
}

#endif // SETTINGS_KEYS_H