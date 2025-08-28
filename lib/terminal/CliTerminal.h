// CliTerminal.h

#ifndef CLI_TERMINAL_H
#define CLI_TERMINAL_H

#include <Arduino.h>
#include <vector>
#include <string>
#include <functional>
#include <map>

// Maximum number of arguments a command can have (including the command itself)
#define MAX_COMMAND_ARGS 10
// Maximum number of commands to store in history
#define MAX_HISTORY_SIZE 15

/**
 * @class CliTerminal
 * @brief An integrated, fully-featured CLI system for the X-Route project.
 *
 * This refactored version includes command history, tab-completion, and a
 * more organized structure, separating declaration from implementation.
 */
class CliTerminal
{
public:
    using CommandCallback = std::function<void(int argc, char *argv[])>;
    using CommandExecutor = std::function<void(const char *commandString)>;

    /**
     * @brief Construct a new CliTerminal object
     * @param port The stream to use for input/output (e.g., Serial).
     * @param executor A function that executes low-level system commands.
     * @param prompt The CLI prompt string to display.
     */
    CliTerminal(Stream &port, CommandExecutor executor, const char *prompt = "xroute> ");

    /**
     * @brief Initializes the terminal, registers commands, and prints a welcome message.
     */
    void begin();

    /**
     * @brief Handles incoming characters from the serial port. Call this in your main loop.
     */
    void handleInput();

private:
    // --- Structures ---
    struct Command
    {
        String name;
        String helpText;
        String usageText;
        String subCommandHelp; // Field for detailed sub-command options
        CommandCallback callback;
    };

    // TrieNode for tab completion
    struct TrieNode
    {
        std::map<char, TrieNode *> children;
        bool isEndOfCommand;
        TrieNode() : isEndOfCommand(false) {}
    };

    // --- Core Members ---
    Stream &_port;
    CommandExecutor _executor;
    String _prompt;
    String _lineBuffer;
    std::vector<Command> _commands;

    // --- Command Registration ---
    void registerCommand(const char *name, const char *helpText, const char *usageText, const char *subCommandHelp, CommandCallback callback);
    void registerInternalCommands();
    void registerAppCommands();

    // --- Command Execution ---
    void executeCommand(String &line);

    // --- Command Handlers (declarations only) ---
    void handleSetSwitch(int argc, char *argv[]);
    void handleSetDimmer(int argc, char *argv[]);
    void handleSetRgb(int argc, char *argv[]);
    void handleSetMotor(int argc, char *argv[]);
    void handleSetWifiSsid(int argc, char *argv[]);
    void handleSetWifiPass(int argc, char *argv[]);
    void handleSetWifiMode(int argc, char *argv[]);
    void handleSetApPass(int argc, char *argv[]);
    void handleSetHostname(int argc, char *argv[]);
    void handleGet(int argc, char *argv[]);
    void handleGetInfo(int argc, char *argv[]);
    void handleCalibrate(int argc, char *argv[]);
    void handleZero(int argc, char *argv[]);
    void handleDefault(int argc, char *argv[]);
    void handleSystem(int argc, char *argv[]);
    void defaultCalibrations();

    // --- Command History ---
    std::vector<String> _history;
    int _historyIndex = 0;
    int _historyCursor = 0;
    void addToHistory(const String &line);
    void recallHistory(int direction);

    // --- Tab Completion ---
    TrieNode *_trieRoot;
    void insertIntoTrie(const String &command);
    void handleTab();
    void findCompletions(TrieNode *node, const String &prefix, std::vector<String> &completions);

    // --- Helper Methods ---
    void printPrompt();
    void clearScreen();
    void printHelp();
    void printCommandHelp(const String &commandName); // Helper for detailed help
    void clearLine();

    // --- Input Processing ---
    void processChar(char c);
    void handleEscapeSequence();
};

#endif // CLI_TERMINAL_H