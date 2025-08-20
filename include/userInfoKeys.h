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

    // new configuration testing
    constexpr char NewConfigIsAvailble[] = "NewConfigIsAvailble";
    constexpr char WifiSSID_NEW[] = "ssid_new";
    constexpr char WifiPassword_NEW[] = "pass_new";
    constexpr char AtempResault[] = "resault";
}

namespace jsonKeys
{
    constexpr char UI_CONFIG[] = "UI_CONFIG";
    constexpr char CONDITON_CONFIG[] = "CONDITION_CONFIG";
    constexpr char WIFI_INFO[] = "WIFI_INFO";
}

namespace otherKeys
{
    constexpr char UI_CONFIG_INDEX[] = "UI_INDEDX";
}

#endif // SETTINGS_KEYS_H