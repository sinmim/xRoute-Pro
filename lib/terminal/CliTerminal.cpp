// CliTerminal.cpp

#include "CliTerminal.h"

// --- Constructor & Initializer ---

CliTerminal::CliTerminal(Stream &port, CommandExecutor executor, const char *prompt)
    : _port(port), _executor(executor), _prompt(prompt), _trieRoot(new TrieNode()) {}

void CliTerminal::begin()
{
    registerInternalCommands();
    registerAppCommands();

    _port.println(F("\n\n==================================="));
    _port.println(F("    X-Route Interactive CLI    "));
    _port.println(F("==================================="));
    _port.println(F("Type 'help' for commands. Use Arrow Keys for history, Tab for completion."));
    printPrompt();
}

// --- Input Handling ---

void CliTerminal::handleInput()
{
    while (_port.available() > 0)
    {
        char c = _port.read();
        processChar(c);
    }
}

void CliTerminal::processChar(char c)
{
    if (c == '\r')
        return; // Ignore carriage return

    if (c == '\n')
    { // Execute command
        _port.println();
        if (_lineBuffer.length() > 0)
        {
            addToHistory(_lineBuffer);
            executeCommand(_lineBuffer);
        }
        _lineBuffer = "";
        printPrompt();
    }
    else if (c == '\b' || c == 127)
    { // Backspace
        if (_lineBuffer.length() > 0)
        {
            _lineBuffer.remove(_lineBuffer.length() - 1);
            _port.print(F("\b \b"));
        }
    }
    else if (c == '\t')
    { // Tab completion
        handleTab();
    }
    else if (c == 27)
    { // Escape sequence (for arrow keys)
        handleEscapeSequence();
    }
    else if (isPrintable(c))
    {
        _lineBuffer += c;
        _port.print(c);
    }
}

void CliTerminal::handleEscapeSequence()
{
    // This handles typical VT100 escape sequences for arrow keys: ESC [ A/B/C/D
    if (!_port.available())
        return;
    char next = _port.read();
    if (next == '[')
    {
        if (!_port.available())
            return;
        char final = _port.read();
        switch (final)
        {
        case 'A':
            recallHistory(-1);
            break; // Up arrow
        case 'B':
            recallHistory(1);
            break; // Down arrow
            // case 'C': // Right arrow - could move cursor
            // case 'D': // Left arrow - could move cursor
        }
    }
}

// --- Command Execution ---

void CliTerminal::executeCommand(String &line)
{
    char line_cstr[line.length() + 1];
    strcpy(line_cstr, line.c_str());

    char *argv[MAX_COMMAND_ARGS];
    int argc = 0;

    char *token = strtok(line_cstr, " ");
    while (token != NULL && argc < MAX_COMMAND_ARGS)
    {
        argv[argc++] = token;
        token = strtok(NULL, " ");
    }

    if (argc == 0)
        return;

    bool found = false;
    for (const auto &cmd : _commands)
    {
        if (cmd.name.equalsIgnoreCase(argv[0]))
        {
            cmd.callback(argc, argv);
            found = true;
            break;
        }
    }

    if (!found)
    {
        _port.print(F("Unknown command: "));
        _port.println(argv[0]);
    }
}

// --- Command History ---

void CliTerminal::addToHistory(const String &line)
{
    if (line.length() == 0 || (_history.size() > 0 && _history.back() == line))
    {
        return; // Don't add empty lines or duplicates
    }
    if (_history.size() >= MAX_HISTORY_SIZE)
    {
        _history.erase(_history.begin());
    }
    _history.push_back(line);
    _historyCursor = _history.size();
}

void CliTerminal::recallHistory(int direction)
{
    if (_history.empty())
        return;

    _historyCursor += direction;

    if (_historyCursor < 0)
        _historyCursor = 0;
    if (_historyCursor >= (int)_history.size())
    {
        _historyCursor = _history.size();
        clearLine();
        _lineBuffer = "";
        return;
    }

    clearLine();
    _lineBuffer = _history[_historyCursor];
    _port.print(_lineBuffer);
}

// --- Tab Completion ---

void CliTerminal::insertIntoTrie(const String &command)
{
    TrieNode *current = _trieRoot;
    for (char c : command)
    {
        if (current->children.find(c) == current->children.end())
        {
            current->children[c] = new TrieNode();
        }
        current = current->children[c];
    }
    current->isEndOfCommand = true;
}

void CliTerminal::findCompletions(TrieNode *node, const String &prefix, std::vector<String> &completions)
{
    if (node->isEndOfCommand)
    {
        completions.push_back(prefix);
    }
    for (auto const &[key, val] : node->children)
    {
        findCompletions(val, prefix + key, completions);
    }
}

void CliTerminal::handleTab()
{
    TrieNode *current = _trieRoot;
    for (char c : _lineBuffer)
    {
        if (current->children.find(c) == current->children.end())
        {
            return; // No completion possible
        }
        current = current->children[c];
    }

    std::vector<String> completions;
    findCompletions(current, _lineBuffer, completions);

    if (completions.size() == 1)
    {
        clearLine();
        _lineBuffer = completions[0] + " ";
        _port.print(_lineBuffer);
    }
    else if (completions.size() > 1)
    {
        _port.println();
        for (const auto &s : completions)
        {
            _port.print(s);
            _port.print("  ");
        }
        _port.println();
        printPrompt();
        _port.print(_lineBuffer);
    }
}

// --- Helper Methods ---

void CliTerminal::printPrompt() { _port.print(_prompt); }
void CliTerminal::clearScreen()
{
    _port.write(27);
    _port.print(F("[2J"));
    _port.write(27);
    _port.print(F("[H"));
}
void CliTerminal::clearLine()
{
    for (unsigned int i = 0; i < _lineBuffer.length(); ++i)
    {
        _port.print(F("\b \b"));
    }
}

void CliTerminal::printHelp()
{
    _port.println(F("\n--- X-Route Command List ---"));
    for (const auto &cmd : _commands)
    {
        _port.print("  ");
        String paddedName = cmd.name;
        paddedName.reserve(16);
        while (paddedName.length() < 16)
            paddedName += " ";
        _port.print(paddedName);
        _port.println(cmd.helpText);

        if (cmd.usageText.length() > 0)
        {
            _port.print("    Usage: ");
            _port.println(cmd.usageText);
        }
    }
    _port.println();
}

void CliTerminal::printCommandHelp(const String &commandName)
{
    bool found = false;
    for (const auto &cmd : _commands)
    {
        if (cmd.name.equalsIgnoreCase(commandName))
        {
            _port.println();
            _port.print(cmd.name);
            _port.print(" - ");
            _port.println(cmd.helpText);

            if (cmd.usageText.length() > 0)
            {
                _port.print("  Usage: ");
                _port.println(cmd.usageText);
            }
            if (cmd.subCommandHelp.length() > 0)
            {
                _port.println("  Options:");
                _port.println(cmd.subCommandHelp);
            }
            _port.println();
            found = true;
            break;
        }
    }
    if (!found)
    {
        _port.print("Help: Unknown command '");
        _port.print(commandName);
        _port.println("'");
    }
}

void CliTerminal::defaultCalibrations()
{
    _port.println(F("--> Restoring all values to their factory defaults."));
    _executor("DEF_ALL_1\n");
}

// --- Command Registration ---

void CliTerminal::registerCommand(const char *name, const char *helpText, const char *usageText, const char *subCommandHelp, CommandCallback callback)
{
    _commands.push_back({name, helpText, usageText, subCommandHelp, callback});
    insertIntoTrie(name);
}

void CliTerminal::registerInternalCommands()
{
    registerCommand("help",
                    "Displays a list of commands or details for a specific command.",
                    "help [command]",
                    "",
                    [this](int argc, char **argv)
                    {
                        if (argc > 1)
                        {
                            this->printCommandHelp(argv[1]);
                        }
                        else
                        {
                            this->printHelp();
                        }
                    });

    registerCommand("clear", "Clears the terminal screen.", "", "", [this](int a, char **v)
                    { this->clearScreen(); });
}

void CliTerminal::registerAppCommands()
{
    using namespace std::placeholders;

    // --- HARDWARE CONTROL ---
    registerCommand("set_sw", "Set switch state.", "set_sw <num> <ON|OFF>", "", std::bind(&CliTerminal::handleSetSwitch, this, _1, _2));
    registerCommand("set_dim", "Set dimmer value.", "set_dim <num> <0-255>", "", std::bind(&CliTerminal::handleSetDimmer, this, _1, _2));
    registerCommand("set_rgb", "Set RGB strip color/brightness.", "set_rgb <num> <r> <g> <b> <bright>", "", std::bind(&CliTerminal::handleSetRgb, this, _1, _2));
    registerCommand("set_motor", "Control a motor.", "set_motor <num> <UP|DOWN|STOP>", "", std::bind(&CliTerminal::handleSetMotor, this, _1, _2));

    // --- WIFI & NETWORK ---
    registerCommand("set_wifi_ssid", "Set WiFi SSID for STA mode.", "set_wifi_ssid <YourSSID>", "", std::bind(&CliTerminal::handleSetWifiSsid, this, _1, _2));
    registerCommand("set_wifi_pass", "Set WiFi password for STA mode.", "set_wifi_pass <YourPassword>", "", std::bind(&CliTerminal::handleSetWifiPass, this, _1, _2));
    registerCommand("set_wifi_mode", "Set WiFi mode (STA, AP, STA_AP).", "set_wifi_mode <STA|AP|STA_AP>", "  <mode>: STA | AP | STA_AP", std::bind(&CliTerminal::handleSetWifiMode, this, _1, _2));
    registerCommand("set_ap_pass", "Set password for AP mode.", "set_ap_pass <password>", "", std::bind(&CliTerminal::handleSetApPass, this, _1, _2));
    registerCommand("set_hostname", "Set the device network hostname.", "set_hostname <new_hostname>", "", std::bind(&CliTerminal::handleSetHostname, this, _1, _2));

    // --- INFO & GETTERS ---
    registerCommand("info", "Get device or WiFi JSON info.", "info <target>", "  <target>: device | wifi", std::bind(&CliTerminal::handleGetInfo, this, _1, _2));
    registerCommand("get", "Get other configs/states.", "get <target> [num]", "  <target>: init | dim_limits | rgb | conditions | ui_config | gyro_ori | coredump", std::bind(&CliTerminal::handleGet, this, _1, _2));

    // --- CALIBRATION ---
    registerCommand("cal", "Calibrate a sensor.", "cal <target> [args...]", "  <target>: voltage | pt100 | int_current | ext_current | gas | max_dim | batt_cap | float | gyro_ori | altitude", std::bind(&CliTerminal::handleCalibrate, this, _1, _2));
    registerCommand("zero", "Set sensor offset to current reading.", "zero <target>", "  <target>: gyro | int_current | ext_current | gas", std::bind(&CliTerminal::handleZero, this, _1, _2));
    registerCommand("default", "Restore setting to factory default.", "default <target>", "  <target>: all | ui_config | conditions | voltage | currents | gas | pt100 | floats | dim_limits | altitude", std::bind(&CliTerminal::handleDefault, this, _1, _2));

    // --- SYSTEM ---
    registerCommand("system", "System operations.", "system <target> [args...]", "  <target>: reset | version | update | del_coredump", std::bind(&CliTerminal::handleSystem, this, _1, _2));
}

// --- Command Handlers (Implementations) ---

void CliTerminal::handleSetSwitch(int argc, char *argv[])
{
    if (argc != 3)
    {
        _port.println(F("Usage: set_sw <num> <ON|OFF>"));
        return;
    }
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "SET_SW_%s=%s\n", argv[1], argv[2]);
    _executor(cmd);
    _port.printf("--> Executed: %s", cmd);
}
void CliTerminal::handleSetDimmer(int argc, char *argv[])
{
    if (argc != 3)
    {
        _port.println(F("Usage: set_dim <num> <0-255>"));
        return;
    }
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "SET_DIM_%s=%s\n", argv[1], argv[2]);
    _executor(cmd);
    _port.printf("--> Executed: %s", cmd);
}
void CliTerminal::handleSetRgb(int argc, char *argv[])
{
    if (argc != 6)
    {
        _port.println(F("Usage: set_rgb <num> <r> <g> <b> <bright 0-255>"));
        return;
    }
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "SET_RGB_%s=%s,%s,%s,%s\n", argv[1], argv[2], argv[3], argv[4], argv[5]);
    _executor(cmd);
    _port.printf("--> Executed: %s", cmd);
}
void CliTerminal::handleSetMotor(int argc, char *argv[])
{
    if (argc != 3)
    {
        _port.println(F("Usage: set_motor <num> <UP|DOWN|STOP>"));
        return;
    }
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "SET_MOT_%s=%s\n", argv[1], argv[2]);
    _executor(cmd);
    _port.printf("--> Executed: %s", cmd);
}
void CliTerminal::handleSetWifiSsid(int argc, char *argv[])
{
    if (argc != 2)
    {
        _port.println(F("Usage: set_wifi_ssid <YourSSID>"));
        return;
    }
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "SAVE_WIFI_SSID_1=%s\n", argv[1]);
    _executor(cmd);
    _port.println(F("--> WiFi SSID set. Restart device to apply."));
}
void CliTerminal::handleSetWifiPass(int argc, char *argv[])
{
    if (argc != 2)
    {
        _port.println(F("Usage: set_wifi_pass <YourPassword>"));
        return;
    }
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "SAVE_WIFI_PASS_1=%s\n", argv[1]);
    _executor(cmd);
    _port.println(F("--> WiFi Password set. Restart device to apply."));
}
void CliTerminal::handleSetWifiMode(int argc, char *argv[])
{
    if (argc != 2)
    {
        printCommandHelp("set_wifi_mode");
        return;
    }
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "SAVE_WIFI_MODE_1=%s\n", argv[1]);
    _executor(cmd);
    _port.println(F("--> WiFi Mode set. Restart device to apply."));
}
void CliTerminal::handleSetApPass(int argc, char *argv[])
{
    if (argc != 2)
    {
        _port.println(F("Usage: set_ap_pass <YourPassword> (min 8 chars)"));
        return;
    }
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "SAVE_WIFI_AP_PASS_1=%s\n", argv[1]);
    _executor(cmd);
    _port.println(F("--> AP Password set. Restart device to apply."));
}
void CliTerminal::handleSetHostname(int argc, char *argv[])
{
    if (argc != 2)
    {
        _port.println(F("Usage: set_hostname <new_hostname>"));
        return;
    }
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "SAVE_HOST_NAME_1=%s\n", argv[1]);
    _executor(cmd);
    _port.println(F("--> Hostname set. Restart device to apply."));
}
void CliTerminal::handleGet(int argc, char *argv[])
{
    if (argc < 2)
    {
        printCommandHelp("get");
        return;
    }
    String target = argv[1];
    char cmd[48];
    if (target.equalsIgnoreCase("init"))
        snprintf(cmd, sizeof(cmd), "GET_INIT_1\n");
    else if (target.equalsIgnoreCase("dim_limits"))
        snprintf(cmd, sizeof(cmd), "GET_DIM_LIM_1\n");
    else if (target.equalsIgnoreCase("rgb"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: get rgb <num>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "GET_RGB_%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("conditions"))
        snprintf(cmd, sizeof(cmd), "GET_CONDITION_CONFIG\n");
    else if (target.equalsIgnoreCase("ui_config"))
        snprintf(cmd, sizeof(cmd), "GET_UI_CONFIG\n");
    else if (target.equalsIgnoreCase("gyro_ori"))
        snprintf(cmd, sizeof(cmd), "GET_GYRO_ORI_1\n");
    else if (target.equalsIgnoreCase("coredump"))
        snprintf(cmd, sizeof(cmd), "GET_CORE_DUMP_1\n");
    else
    {
        _port.println(F("Invalid 'get' target."));
        printCommandHelp("get");
        return;
    }
    _executor(cmd);
    _port.printf("--> Request sent.\n");
}
void CliTerminal::handleGetInfo(int argc, char *argv[])
{
    if (argc != 2)
    {
        printCommandHelp("info");
        return;
    }
    String target = argv[1];
    if (target.equalsIgnoreCase("device"))
        _executor("GET_DEVICE_INFO_JSON_1\n");
    else if (target.equalsIgnoreCase("wifi"))
        _executor("GET_WIFI_INFO_JSON_1\n");
    else
    {
        _port.println(F("Invalid 'info' target."));
        printCommandHelp("info");
        return;
    }
}
void CliTerminal::handleCalibrate(int argc, char *argv[])
{
    if (argc < 2)
    {
        printCommandHelp("cal");
        return;
    }
    String target = argv[1];
    char cmd[64];
    if (target.equalsIgnoreCase("voltage"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal voltage <value_mV>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_VLT_1=%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("pt100"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal pt100 <temp_x10 C> (e.g. 25.5C is 255)"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_OTMP_1=%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("int_current"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal int_current <value_mA>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_INT_CUR_1=%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("ext_current"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal ext_current <value_mA>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_EXT_CUR_1=%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("gas"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal gas <value>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_GAS_1=%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("max_dim"))
    {
        if (argc != 4)
        {
            _port.println(F("Usage: cal max_dim <num> <0-255>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "SET_MAX_DIM_%s=%s\n", argv[2], argv[3]);
    }
    else if (target.equalsIgnoreCase("batt_cap"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal batt_cap <Ah>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_BATT_CAP_1=%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("float"))
    {
        if (argc != 4)
        {
            _port.println(F("Usage: cal float <num> <min|max>"));
            return;
        }
        String type = argv[3];
        if (type.equalsIgnoreCase("min"))
            snprintf(cmd, sizeof(cmd), "CAL_MIN_FLT_%s\n", argv[2]);
        else
            snprintf(cmd, sizeof(cmd), "CAL_MAX_FLT_%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("gyro_ori"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal gyro_ori <XY01|...>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_GYRO_ORI_1=%s\n", argv[2]);
    }
    else if (target.equalsIgnoreCase("altitude"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: cal altitude <meters>"));
            return;
        }
        snprintf(cmd, sizeof(cmd), "CAL_ALT_1=%s\n", argv[2]);
    }
    else
    {
        _port.println(F("Invalid 'cal' target."));
        printCommandHelp("cal");
        return;
    }
    _executor(cmd);
    _port.printf("--> Calibration command sent.\n");
}
void CliTerminal::handleZero(int argc, char *argv[])
{
    if (argc != 2)
    {
        printCommandHelp("zero");
        return;
    }
    String target = argv[1];
    char cmd[48];
    if (target.equalsIgnoreCase("gyro"))
        snprintf(cmd, sizeof(cmd), "SET_ZERO_GYRO_1\n");
    else if (target.equalsIgnoreCase("int_current"))
        snprintf(cmd, sizeof(cmd), "CAL_ZERO_INT_CUR_1\n");
    else if (target.equalsIgnoreCase("ext_current"))
        snprintf(cmd, sizeof(cmd), "CAL_ZERO_EXT_CUR_1\n");
    else if (target.equalsIgnoreCase("gas"))
        snprintf(cmd, sizeof(cmd), "CAL_ZERO_GAS_1\n");
    else
    {
        _port.println(F("Invalid 'zero' target."));
        printCommandHelp("zero");
        return;
    }
    _executor(cmd);
    _port.printf("--> Zeroing command sent.\n");
}
void CliTerminal::handleDefault(int argc, char *argv[])
{
    if (argc != 2)
    {
        printCommandHelp("default");
        return;
    }
    String target = argv[1];
    char cmd[48] = {0};
    if (target.equalsIgnoreCase("ui_config"))
        snprintf(cmd, sizeof(cmd), "DEF_UICNFG_1\n");
    else if (target.equalsIgnoreCase("conditions"))
        snprintf(cmd, sizeof(cmd), "DEF_CNDTION_1\n");
    else if (target.equalsIgnoreCase("voltage"))
        snprintf(cmd, sizeof(cmd), "DEF_VLT_1\n");
    else if (target.equalsIgnoreCase("currents"))
    {
        _executor("DEF_INT_CUR_1\n");
        _executor("DEF_EXT_CUR_1\n");
        _port.println(F("--> Internal and External current defaults restored."));
        return;
    }
    else if (target.equalsIgnoreCase("gas"))
        snprintf(cmd, sizeof(cmd), "DEF_GAS_1\n");
    else if (target.equalsIgnoreCase("pt100"))
        snprintf(cmd, sizeof(cmd), "DEF_OTMP_1\n");
    else if (target.equalsIgnoreCase("floats"))
    {
        _executor("DEF_FLT_1\n");
        _executor("DEF_FLT_2\n");
        _executor("DEF_FLT_3\n");
        _port.println(F("--> All float sensor defaults restored."));
        return;
    }
    else if (target.equalsIgnoreCase("dim_limits"))
        snprintf(cmd, sizeof(cmd), "DEF_DIM_LIMITS_1\n");
    else if (target.equalsIgnoreCase("altitude"))
        snprintf(cmd, sizeof(cmd), "DEF_ALT_1\n");
    else if (target.equalsIgnoreCase("all"))
    {
        _port.println(F("Restoring ALL settings to factory defaults..."));
        defaultCalibrations();
        return;
    }
    else
    {
        _port.println(F("Invalid 'default' target."));
        printCommandHelp("default");
        return;
    }
    _executor(cmd);
    _port.printf("--> Default values restored for %s.\n", target.c_str());
}
void CliTerminal::handleSystem(int argc, char *argv[])
{
    if (argc < 2)
    {
        printCommandHelp("system");
        return;
    }
    String target = argv[1];
    if (target.equalsIgnoreCase("reset"))
    {
        _port.println(F("Resetting device..."));
        _executor("SET_DEV_RESET_1\n");
    }
    else if (target.equalsIgnoreCase("version"))
    {
        _executor("GET_VER_1\n");
    }
    else if (target.equalsIgnoreCase("update"))
    {
        if (argc != 3)
        {
            _port.println(F("Usage: system update <binary_size_bytes>"));
            return;
        }
        char cmd[48];
        snprintf(cmd, sizeof(cmd), "SET_START_UPDATE_1=%s\n", argv[2]);
        _executor(cmd);
    }
    else if (target.equalsIgnoreCase("del_coredump"))
    {
        _executor("SET_DEL_CORE_DUMP_1\n");
    }
    else
    {
        _port.println(F("Invalid 'system' command."));
        printCommandHelp("system");
    }
}