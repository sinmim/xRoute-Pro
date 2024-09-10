const char *defaultCondition = R"(
{
  "conditions" : [
    {
      "inType" : "REL",
      "inPort" : 1,
      "opr" : "=",
      "setpoint" : 1,
      "outType" : "DIM"
      "outPort" : 2,
      "outVal" : 66,
    }
  ],
  "timerConditions : [
    {
      "type" : "DIM" 
      "port" : 1,
      "uTime":5000,
      "dTime":5000
    }
  ]
}
)";
