#ifndef jsonCondition_H
#define jsonCondition_H
#include <ArduinoJson.h>
#include <SPIFFS.h>
void setcondCreatorFunction(void (*func)(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue ));

class ConditionsReader
{
public:
    ConditionsReader();
    void readJsonConditionsFromFile(String filename);
    void saveDefaultConditions(String filename);
    void saveConditionsFileFromString(String filename,String str);
    static int conditionCount;

private:
    StaticJsonDocument<2048> doc;
};

#endif // CONFIG_READER_H
