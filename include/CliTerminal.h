#ifndef CLI_TERMINAL_H
#define CLI_TERMINAL_H

#include <Arduino.h>
#include <vector>
#include <string>
#include <functional>

// Maximum number of arguments a command can have (including the command itself)
#define MAX_COMMAND_ARGS 10

/**
 * @class CliTerminal
 * @brief An integrated, fully-featured CLI system for the X-Route project.
 *
 * This version exposes all key commands, including WiFi settings, at the top level
 * for better visibility in the 'help' menu.
 */
class CliTerminal {
public:
    using CommandCallback = std::function<void(int argc, char* argv[])>;
    using CommandExecutor = std::function<void(const char* commandString)>;

    CliTerminal(Stream& port, CommandExecutor executor, const char* prompt = "xroute> ")
        : _port(port), _executor(executor), _prompt(prompt) {}

    void begin() {
        registerInternalCommands();
        registerAppCommands();

        _port.println(F("\n\n==================================="));
        _port.println(F("    X-Route Interactive CLI    "));
        _port.println(F("==================================="));
        _port.println(F("Type 'help' for a list of commands."));
        printPrompt();
    }

    void handleInput() {
        while (_port.available() > 0) {
            char c = _port.read();
            if (c == '\r') continue;
            if (c == '\n') {
                _port.println();
                if (_lineBuffer.length() > 0) {
                    executeCommand(_lineBuffer);
                    _lineBuffer = "";
                }
                printPrompt();
            } else if (c == '\b' || c == 127) {
                if (_lineBuffer.length() > 0) {
                    _lineBuffer.remove(_lineBuffer.length() - 1);
                    _port.print(F("\b \b"));
                }
            } else if (isPrintable(c)) {
                _lineBuffer += c;
                _port.print(c);
            }
        }
    }

private:
    struct Command { String name; String helpText; CommandCallback callback; };
    Stream& _port;
    CommandExecutor _executor;
    String _prompt;
    String _lineBuffer;
    std::vector<Command> _commands;

    void registerCommand(const char* name, const char* helpText, CommandCallback callback) {
        _commands.push_back({name, helpText, callback});
    }

    // --- Command Handlers (Refactored for better discoverability) ---

    // SET Commands
    void handleSetSwitch(int argc, char *argv[]) { if (argc != 3) { _port.println(F("Usage: set_sw <num> <ON|OFF>")); return; } char cmd[32]; snprintf(cmd, sizeof(cmd), "SET_SW_%s=%s\n", argv[1], argv[2]); _executor(cmd); _port.printf("--> Executed: %s", cmd); }
    void handleSetDimmer(int argc, char *argv[]) { if (argc != 3) { _port.println(F("Usage: set_dim <num> <0-255>")); return; } char cmd[32]; snprintf(cmd, sizeof(cmd), "SET_DIM_%s=%s\n", argv[1], argv[2]); _executor(cmd); _port.printf("--> Executed: %s", cmd); }
    void handleSetRgb(int argc, char *argv[]) { if (argc != 6) { _port.println(F("Usage: set_rgb <num> <r> <g> <b> <bright 0-255>")); return; } char cmd[64]; snprintf(cmd, sizeof(cmd), "SET_RGB_%s=%s,%s,%s,%s\n", argv[1], argv[2], argv[3], argv[4], argv[5]); _executor(cmd); _port.printf("--> Executed: %s", cmd); }
    void handleSetMotor(int argc, char *argv[]) { if (argc != 3) { _port.println(F("Usage: set_motor <num> <UP|DOWN|STOP>")); return; } char cmd[32]; snprintf(cmd, sizeof(cmd), "SET_MOT_%s=%s\n", argv[1], argv[2]); _executor(cmd); _port.printf("--> Executed: %s", cmd); }

    // WIFI Configuration Commands
    void handleSetWifiSsid(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: set_wifi_ssid <YourSSID>")); return; } char cmd[128]; snprintf(cmd, sizeof(cmd), "SAVE_WIFI_SSID_1=%s\n", argv[1]); _executor(cmd); _port.println(F("--> WiFi SSID set. Restart device to apply.")); }
    void handleSetWifiPass(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: set_wifi_pass <YourPassword>")); return; } char cmd[128]; snprintf(cmd, sizeof(cmd), "SAVE_WIFI_PASS_1=%s\n", argv[1]); _executor(cmd); _port.println(F("--> WiFi Password set. Restart device to apply.")); }
    void handleSetWifiMode(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: set_wifi_mode <STA|AP|STA_AP>")); return; } char cmd[64]; snprintf(cmd, sizeof(cmd), "SAVE_WIFI_MODE_1=%s\n", argv[1]); _executor(cmd); _port.println(F("--> WiFi Mode set. Restart device to apply.")); }
    void handleSetApPass(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: set_ap_pass <YourPassword> (min 8 chars)")); return; } char cmd[128]; snprintf(cmd, sizeof(cmd), "SAVE_WIFI_AP_PASS_1=%s\n", argv[1]); _executor(cmd); _port.println(F("--> AP Password set. Restart device to apply.")); }
    void handleSetHostname(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: set_hostname <new_hostname>")); return; } char cmd[128]; snprintf(cmd, sizeof(cmd), "SAVE_HOST_NAME_1=%s\n", argv[1]); _executor(cmd); _port.println(F("--> Hostname set. Restart device to apply.")); }

    // GET Commands
    void handleGet(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: get <init|dim_limits|rgb|conditions|ui_config|gyro_ori|coredump> [num]")); return; } String target = argv[1]; char cmd[48]; if (target.equalsIgnoreCase("init")) snprintf(cmd, sizeof(cmd), "GET_INIT_1\n"); else if (target.equalsIgnoreCase("dim_limits")) snprintf(cmd, sizeof(cmd), "GET_DIM_LIM_1\n"); else if (target.equalsIgnoreCase("rgb")) { if (argc != 3) {_port.println(F("Usage: get rgb <num>")); return;} snprintf(cmd, sizeof(cmd), "GET_RGB_%s\n", argv[2]); } else if (target.equalsIgnoreCase("conditions")) snprintf(cmd, sizeof(cmd), "GET_CONDITION_CONFIG\n"); else if (target.equalsIgnoreCase("ui_config")) snprintf(cmd, sizeof(cmd), "GET_UI_CONFIG\n"); else if (target.equalsIgnoreCase("gyro_ori")) snprintf(cmd, sizeof(cmd), "GET_GYRO_ORI_1\n"); else if (target.equalsIgnoreCase("coredump")) snprintf(cmd, sizeof(cmd), "GET_CORE_DUMP_1\n"); else { _port.println(F("Invalid 'get' target.")); return; } _executor(cmd); _port.printf("--> Request sent.\n"); }
    void handleGetInfo(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: info <device|wifi>")); return; } String target = argv[1]; if (target.equalsIgnoreCase("device")) _executor("GET_DEVICE_INFO_JSON_1\n"); else if (target.equalsIgnoreCase("wifi")) _executor("GET_WIFI_INFO_JSON_1\n"); else { _port.println(F("Invalid 'info' target.")); return; } }
    
    // CALIBRATE and ZERO Commands (same as before)
    void handleCalibrate(int argc, char* argv[]) { if (argc < 2) { _port.println(F("Usage: cal <target> [args...]")); return; } String target = argv[1]; char cmd[64]; if (target.equalsIgnoreCase("voltage")) { if(argc!=3){_port.println(F("Usage: cal voltage <value_mV>")); return;} snprintf(cmd, sizeof(cmd), "CAL_VLT_1=%s\n", argv[2]); } else if (target.equalsIgnoreCase("pt100")) { if(argc!=3){_port.println(F("Usage: cal pt100 <temp_x10 C> (e.g. 25.5C is 255)")); return;} snprintf(cmd, sizeof(cmd), "CAL_OTMP_1=%s\n", argv[2]); } else if (target.equalsIgnoreCase("int_current")) { if(argc!=3){_port.println(F("Usage: cal int_current <value_mA>")); return;} snprintf(cmd, sizeof(cmd), "CAL_INT_CUR_1=%s\n", argv[2]); } else if (target.equalsIgnoreCase("ext_current")) { if(argc!=3){_port.println(F("Usage: cal ext_current <value_mA>")); return;} snprintf(cmd, sizeof(cmd), "CAL_EXT_CUR_1=%s\n", argv[2]); } else if (target.equalsIgnoreCase("gas")) { if(argc!=3){_port.println(F("Usage: cal gas <value>")); return;} snprintf(cmd, sizeof(cmd), "CAL_GAS_1=%s\n", argv[2]); } else if (target.equalsIgnoreCase("max_dim")) { if(argc!=4){_port.println(F("Usage: cal max_dim <num> <0-255>")); return;} snprintf(cmd, sizeof(cmd), "CAL_MAX_DIM_%s=%s\n", argv[2], argv[3]); } else if (target.equalsIgnoreCase("batt_cap")) { if(argc!=3){_port.println(F("Usage: cal batt_cap <Ah>")); return;} snprintf(cmd, sizeof(cmd), "CAL_BATT_CAP_1=%s\n", argv[2]); } else if (target.equalsIgnoreCase("float")) { if(argc!=4){_port.println(F("Usage: cal float <num> <min|max>")); return;} String type=argv[3]; if(type.equalsIgnoreCase("min")) snprintf(cmd, sizeof(cmd), "CAL_MIN_FLT_%s\n", argv[2]); else snprintf(cmd, sizeof(cmd), "CAL_MAX_FLT_%s\n", argv[2]); } else if (target.equalsIgnoreCase("gyro_ori")) { if(argc!=3){_port.println(F("Usage: cal gyro_ori <XY01|...>")); return;} snprintf(cmd, sizeof(cmd), "CAL_GYRO_ORI_1=%s\n", argv[2]); } else if (target.equalsIgnoreCase("altitude")) { if(argc!=3){_port.println(F("Usage: cal altitude <meters>")); return;} snprintf(cmd, sizeof(cmd), "CAL_ALT_1=%s\n", argv[2]); } else { _port.println(F("Invalid 'cal' target.")); return; } _executor(cmd); _port.printf("--> Calibration command sent.\n"); }
    void handleZero(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: zero <gyro|int_current|ext_current|gas>")); return; } String target = argv[1]; char cmd[48]; if (target.equalsIgnoreCase("gyro")) snprintf(cmd, sizeof(cmd), "SET_ZERO_GYRO_1\n"); else if (target.equalsIgnoreCase("int_current")) snprintf(cmd, sizeof(cmd), "CAL_ZERO_INT_CUR_1\n"); else if (target.equalsIgnoreCase("ext_current")) snprintf(cmd, sizeof(cmd), "CAL_ZERO_EXT_CUR_1\n"); else if (target.equalsIgnoreCase("gas")) snprintf(cmd, sizeof(cmd), "CAL_ZERO_GAS_1\n"); else { _port.println(F("Invalid 'zero' target.")); return; } _executor(cmd); _port.printf("--> Zeroing command sent.\n"); }

    // DEFAULT Command
    void handleDefault(int argc, char* argv[]) { if (argc != 2) { _port.println(F("Usage: default <all|ui_config|conditions|voltage|currents|gas|pt100|floats|dim_limits|altitude>")); return; } String target = argv[1]; char cmd[48] = {0}; if (target.equalsIgnoreCase("ui_config")) snprintf(cmd, sizeof(cmd), "DEF_UICNFG_1\n"); else if (target.equalsIgnoreCase("conditions")) snprintf(cmd, sizeof(cmd), "DEF_CNDTION_1\n"); else if (target.equalsIgnoreCase("voltage")) snprintf(cmd, sizeof(cmd), "DEF_VLT_1\n"); else if (target.equalsIgnoreCase("currents")) { _executor("DEF_INT_CUR_1\n"); _executor("DEF_EXT_CUR_1\n"); _port.println(F("--> Internal and External current defaults restored.")); return; } else if (target.equalsIgnoreCase("gas")) snprintf(cmd, sizeof(cmd), "DEF_GAS_1\n"); else if (target.equalsIgnoreCase("pt100")) snprintf(cmd, sizeof(cmd), "DEF_OTMP_1\n"); else if (target.equalsIgnoreCase("floats")) { _executor("DEF_FLT_1\n");_executor("DEF_FLT_2\n");_executor("DEF_FLT_3\n"); _port.println(F("--> All float sensor defaults restored.")); return; } else if (target.equalsIgnoreCase("dim_limits")) snprintf(cmd, sizeof(cmd), "DEF_DIM_LIMITS_1\n"); else if (target.equalsIgnoreCase("altitude")) snprintf(cmd, sizeof(cmd), "DEF_ALT_1\n"); else if (target.equalsIgnoreCase("all")) { _port.println(F("Restoring ALL settings to factory defaults...")); defaultCalibrations(); return; } else { _port.println(F("Invalid 'default' target.")); return; } _executor(cmd); _port.printf("--> Default values restored for %s.\n", target.c_str()); }

    // SYSTEM Commands
    void handleSystem(int argc, char* argv[]) { if (argc < 2) { _port.println(F("Usage: system <reset|version|update|del_coredump> [args...]")); return; } String target = argv[1]; if (target.equalsIgnoreCase("reset")) { _port.println(F("Resetting device...")); _executor("SET_DEV_RESET_1\n"); } else if (target.equalsIgnoreCase("version")) { _executor("GET_VER_1\n"); } else if (target.equalsIgnoreCase("update")) { if (argc != 3) { _port.println(F("Usage: system update <binary_size_bytes>")); return; } char cmd[48]; snprintf(cmd, sizeof(cmd), "SET_START_UPDATE_1=%s\n", argv[2]); _executor(cmd); } else if (target.equalsIgnoreCase("del_coredump")) { _executor("SET_DEL_CORE_DUMP_1\n"); } else { _port.println(F("Invalid 'system' command.")); } }

    // --- Command Registration ---
    void registerInternalCommands() {
        registerCommand("help", "Displays this help message.", [this](int a, char** v){ this->printHelp(); });
        registerCommand("clear", "Clears the terminal screen.", [this](int a, char** v){ this->clearScreen(); });
    }

    void registerAppCommands() {
        using namespace std::placeholders;
        
        // --- HARDWARE CONTROL ---
        registerCommand("set_sw", "Set switch state. Ex: set_sw 1 ON", std::bind(&CliTerminal::handleSetSwitch, this, _1, _2));
        registerCommand("set_dim", "Set dimmer value. Ex: set_dim 1 255", std::bind(&CliTerminal::handleSetDimmer, this, _1, _2));
        registerCommand("set_rgb", "Set RGB strip. Ex: set_rgb 1 255 0 0 128", std::bind(&CliTerminal::handleSetRgb, this, _1, _2));
        registerCommand("set_motor", "Control a motor. Ex: set_motor 1 UP", std::bind(&CliTerminal::handleSetMotor, this, _1, _2));

        // --- WIFI & NETWORK ---
        registerCommand("set_wifi_ssid", "Set WiFi SSID for STA mode.", std::bind(&CliTerminal::handleSetWifiSsid, this, _1, _2));
        registerCommand("set_wifi_pass", "Set WiFi password for STA mode.", std::bind(&CliTerminal::handleSetWifiPass, this, _1, _2));
        registerCommand("set_wifi_mode", "Set WiFi mode (STA, AP, STA_AP).", std::bind(&CliTerminal::handleSetWifiMode, this, _1, _2));
        registerCommand("set_ap_pass", "Set password for AP mode.", std::bind(&CliTerminal::handleSetApPass, this, _1, _2));
        registerCommand("set_hostname", "Set the device network hostname.", std::bind(&CliTerminal::handleSetHostname, this, _1, _2));

        // --- INFO & GETTERS ---
        registerCommand("info", "Get device or WiFi info. Ex: info wifi", std::bind(&CliTerminal::handleGetInfo, this, _1, _2));
        registerCommand("get", "Get other configs/states.", std::bind(&CliTerminal::handleGet, this, _1, _2));
        
        // --- CALIBRATION ---
        registerCommand("cal", "Calibrate a sensor. Ex: cal voltage 12500", std::bind(&CliTerminal::handleCalibrate, this, _1, _2));
        registerCommand("zero", "Set sensor offset to current reading.", std::bind(&CliTerminal::handleZero, this, _1, _2));
        registerCommand("default", "Restore setting to factory default.", std::bind(&CliTerminal::handleDefault, this, _1, _2));

        // --- SYSTEM ---
        registerCommand("system", "System ops. Ex: system reset", std::bind(&CliTerminal::handleSystem, this, _1, _2));
    }

    // --- Helper Methods ---
    void printPrompt() { _port.print(_prompt); }
    void clearScreen() { _port.write(27); _port.print(F("[2J")); _port.write(27); _port.print(F("[H")); }
    void printHelp() { _port.println(F("\n--- X-Route Command List ---")); for (const auto& cmd : _commands) { _port.print("  "); _port.print(cmd.name); for (int i = cmd.name.length(); i < 16; ++i) { _port.print(" "); } _port.println(cmd.helpText); } _port.println(); }
    void executeCommand(String& line) { char* argv[MAX_COMMAND_ARGS]; int argc = 0; char line_cstr[line.length() + 1]; strcpy(line_cstr, line.c_str()); char* token = strtok(line_cstr, " "); while (token != NULL && argc < MAX_COMMAND_ARGS) { argv[argc++] = token; token = strtok(NULL, " "); } if (argc == 0) return; bool found = false; for (const auto& cmd : _commands) { if (cmd.name.equalsIgnoreCase(argv[0])) { cmd.callback(argc, argv); found = true; break; } } if (!found) { _port.print(F("Unknown command: ")); _port.println(argv[0]); } }
    void defaultCalibrations() { _port.println(F("--> Restoring all values to their factory defaults.")); _executor("DEF_ALL_1\n"); }
};

#endif // CLI_TERMINAL_H