const char *defaultCondition = R"(
{
  "conditions": [
    {
      "inType": "FLOATER",
      "inPort": 0,
      "opr": ">",
      "setpoint": 800,
      "outType": "DIM",
      "outPort": 1,
      "outVal": 10
    },
    {
      "inType": "FLOATER",
      "inPort": 0,
      "opr": "<",
      "setpoint": 800,
      "outType": "DIM",
      "outPort": 1,
      "outVal": 0
    }
  ]
}
)";
