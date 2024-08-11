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
    },
    {
      "inType": "DIM",
      "inPort": 1,
      "opr": ">",
      "setpoint": 80,
      "outType": "DIM",
      "outPort": 2,
      "outVal": 100
    },
    {
      "inType": "DIM",
      "inPort": 1,
      "opr": "<",
      "setpoint": 80,
      "outType": "DIM",
      "outPort": 2,
      "outVal": 0
    }
  ]
}
)";
