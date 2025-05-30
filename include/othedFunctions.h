#ifndef othedFunctions_h
#define othedFunctions_h
void generatePass(char *result);
bool checkPass(uint64_t uid, const char *pass);
void SendToAll(char *str);
void SendToAll(const char *str);
void SerialPrint(char *str);
double psiToMeters(double x);
double ACID_SOC_OCV(double V);
double AGM_SOC_OCV(double v);
void printTaskResourceUsage(int interval);
void giveMeMacAdress();
bool SaveStringToFile(String str, String path);
String readStringFromFile(String path);

#endif