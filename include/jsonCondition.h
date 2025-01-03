#ifndef jsonCondition_H
#define jsonCondition_H
#include <ArduinoJson.h>
#include <SPIFFS.h>
void setCondCreatorFunction(void (*func)(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue));
void setTimerCondCreatorFunction(void (*func)(String _outputType,int _motorPort, int _upTimeMs, int downTimeMs));
class ConditionsReader
{
public:
    ConditionsReader();
    bool readJsonConditionsFromFile(String filename);
    void saveDefaultConditions(String filename);
    void saveConditionsFileFromString(String filename, String str);
    static int conditionCount;
    bool isJsonFileOk(String filename);

private:
    StaticJsonDocument<4096> doc;
};

#endif // CONFIG_READER_H
