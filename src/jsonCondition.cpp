#include "jsonCondition.h"
#include "SPIFFS.h"
#include "defaultConditions.h"
extern const char *defaultCondition;

int ConditionsReader::conditionCount = 0;

ConditionsReader::ConditionsReader()
{
  
}

void (*condCreator)(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue);
void setcondCreatorFunction(void (*func)(String _inputType, int _inputPort, String _oprt, float _setpoint, String _outputType, int _outputPort, int _outputValue))
{
  condCreator = func;
}

void ConditionsReader::saveDefaultConditions(String filename)
{
  File file = SPIFFS.open(filename.c_str(), FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }

  int bytesWritten = file.print(defaultCondition);
  Serial.printf("Written %d bytes to file: %s\n", bytesWritten, filename.c_str());
  file.close();

  // Reopen the file for reading to verify contents
  file = SPIFFS.open(filename.c_str(), FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  String fileContent = file.readString();
  Serial.println("File content:");
  Serial.println(fileContent);
  file.close();
}

void ConditionsReader::saveConditionsFileFromString(String filename, String str)
{
  File file = SPIFFS.open(filename.c_str(), FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  Serial.println("Str to save:" + str);

  int bytesWritten = file.print(str);
  Serial.printf("Written %d bytes to file: %s\n", bytesWritten, filename.c_str());
  file.close();

  // Reopen the file for reading to verify contents
  file = SPIFFS.open(filename.c_str(), FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  String fileContent = file.readString();
  Serial.println("Saved File content:");
  Serial.println(fileContent);
  file.close();
}

void ConditionsReader::readJsonConditionsFromFile(String filename)
{
  // Check if the file exists
  if (!SPIFFS.exists(filename))
  {
    Serial.println("File does not exist: " + filename);
    return;
  }

  // Open the file
  File file = SPIFFS.open(filename.c_str(), FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file: " + filename);
    return;
  }

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
  {
    Serial.print(F("Failed to read file, using default configuration: "));
    Serial.println(error.c_str());
    file.close();
    return;
  }
  file.close();
  // Parse buttons
  JsonArray conditionArray = doc["conditions"];
  ConditionsReader::conditionCount = conditionArray.size();

  for (size_t i = 0; i < conditionCount; i++)
  {
    JsonObject condition = conditionArray[i];
    // strlcpy(buttons[i].name, condition["name"] | "", sizeof(buttons[i].name));
    // buttons[i].port = condition["port"];
    // buttons[i].image = condition["image"];
    String _inputType = condition["inType"];
    int _inputPort = condition["inPrt"];
    String _oprt = condition["oprt"];
    float _setpoint = condition["setPnt"];
    String _outputType = condition["outType"];
    int _outputPort = condition["outputPort"];
    int _outputValue = condition["outputValue"];
    condCreator(_inputType, _inputPort, _oprt, _setpoint, _outputType, _outputPort, _outputValue);
  }

  Serial.println("Conditions loaded successfully from file: " + filename);
}