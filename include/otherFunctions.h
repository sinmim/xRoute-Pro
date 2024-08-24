#ifndef otherFunctions_h
#define otherFunctions_h
void SetNextion(unsigned int pos, float *dimTmp, float *dimLimit);
void generatePass(char *result);
bool checkPass(uint64_t uid, const char *pass);
void sendToAll(char *str);
void sendToAll(const char *str);
void sendToAll(String str);
void SerialPrint(char *str);
double psiToMeters(double x);
double ACID_SOC_OCV(double V);
double AGM_SOC_OCV(double v);
void printTaskResourceUsage(int interval);
void giveMeMacAdress();
bool SaveStringToFile(String str, String path);
String readStringFromFile(String path);
int getDimVal(int n);

#endif