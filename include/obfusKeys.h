#ifndef _OBFUSKEYS_H
#define _OBFUSKEYS_H

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
// for keys run this code on online compiler
/*
#include <iostream>
#include <vector>
#include <string>
#include <cstring> // for strlen

// Rolling XOR obfuscation that returns a vector of bytes.
std::vector<unsigned char> obfuscate_rolling(const char* input, unsigned char seed) {
    size_t len = std::strlen(input);
    std::vector<unsigned char> obf(len);

    unsigned char current = seed;
    for (size_t i = 0; i < len; i++) {
        unsigned char c = static_cast<unsigned char>(input[i]);
        unsigned char obfChar = c ^ current;
        obf[i] = obfChar;
        current = obfChar;
    }
    return obf;
}

// Rolling XOR deobfuscation that takes a vector of obfuscated bytes.
std::string deobfuscate_rolling(const std::vector<unsigned char>& obf, unsigned char seed) {
    std::string original;
    original.resize(obf.size());

    unsigned char prev = seed;
    for (size_t i = 0; i < obf.size(); i++) {
        unsigned char obfChar = obf[i];
        unsigned char origChar = obfChar ^ prev;
        original[i] = static_cast<char>(origChar);
        prev = obfChar;
    }
    return original;
}

int main() {
    const char* message = "Z3dPD";
    unsigned char seed = 0x5A;

    std::vector<unsigned char> obfuscated = obfuscate_rolling(message, seed);

    // Print obfuscated bytes in the desired format
    std::cout << "Obfuscated bytes: {";
    for (size_t i = 0; i < obfuscated.size(); i++) {
        printf("0x%02X", obfuscated[i]);
        if (i < obfuscated.size() - 1) {
            std::cout << ",";
        }
    }
    std::cout << "}" << std::endl;

    // Deobfuscate
    std::string recovered = deobfuscate_rolling(obfuscated, seed);
    std::cout << "Recovered text: " << recovered << std::endl;

    return 0;
}
*/
// -----------------------------------------------------------------------------
// Global seed for rolling XOR. You can change this if you like.
static const unsigned char seed = 0x5A;
// --------------------   Work     Gyro     Hum    Current    Gas
// xrtLcns = new RegDev("K3nY0", "8px7n", "tQ5Eb", "pdjtk", "Z3dPD", RegFilePath);
// -----------------------------------------------------------------------------
// Example obfuscated byte arrays for your 5 keys.
// Each array can contain any bytes, including 0x00.
static const unsigned char obfWorKey[] = {0x11, 0x22, 0x4C, 0x15, 0x25};
static const unsigned char obfGyrKey[] = {0x62, 0x12, 0x6A, 0x5D, 0x33};
static const unsigned char obfHumKey[] = {0x2E, 0x7F, 0x4A, 0x0F, 0x6D};
static const unsigned char obfCurKey[] = {0x2A, 0x4E, 0x24, 0x50, 0x3B};
static const unsigned char obfGasKey[] = {0x00, 0x33, 0x57, 0x07, 0x43};

// -----------------------------------------------------------------------------
// Rolling XOR Obfuscation
//  - input: a null-terminated C-string to be obfuscated
//  - seedVal: initial seed for rolling XOR (defaults to global 'seed')
// Returns a std::vector<unsigned char> so we don't lose any 0x00 bytes.
inline std::vector<unsigned char> obfuscate_rolling(const char *input, unsigned char seedVal = seed)
{
    size_t len = std::strlen(input);
    std::vector<unsigned char> obf(len);

    unsigned char current = seedVal;
    for (size_t i = 0; i < len; i++)
    {
        unsigned char c = static_cast<unsigned char>(input[i]);
        unsigned char obfChar = c ^ current;
        obf[i] = obfChar;
        current = obfChar; // rolling update
    }
    return obf;
}

// -----------------------------------------------------------------------------
// Rolling XOR Deobfuscation
//  - obf: a vector of obfuscated bytes
//  - seedVal: the same initial seed that was used to obfuscate
// Returns a std::string with the recovered plaintext.
inline std::string deobfuscate_rolling(const std::vector<unsigned char> &obf, unsigned char seedVal = seed)
{
    std::string original(obf.size(), '\0');

    unsigned char prev = seedVal;
    for (size_t i = 0; i < obf.size(); i++)
    {
        unsigned char obfChar = obf[i];
        unsigned char origChar = obfChar ^ prev;
        original[i] = static_cast<char>(origChar);
        prev = obfChar; // rolling update
    }
    return original;
}

// -----------------------------------------------------------------------------
// Overload: Deobfuscate directly from a fixed-size array of bytes.
// This is useful if your keys are stored as static const arrays (like obfWorkKey).
template <size_t N>
inline std::string deobfuscate_rolling(const unsigned char (&obfArray)[N], unsigned char seedVal = seed)
{
    // Convert the fixed array to a vector, then use the main function above.
    std::vector<unsigned char> obf(obfArray, obfArray + N);
    return deobfuscate_rolling(obf, seedVal);
}

#endif // _OBFUSKEYS_H
