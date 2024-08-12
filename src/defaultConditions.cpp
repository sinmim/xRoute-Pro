const char *defaultCondition = R"(
{
  "conditions" : [
    {
      "inPort" : 1,
      "inType" : "REL",
      "setpoint" : 1,
      "opr" : "=",
      "outPort" : 2,
      "outVal" : 66,
      "outType" : "DIM"
    },
    {
      "inPort" : 1,
      "inType" : "REL",
      "outType" : "DIM",
      "opr" : "=",
      "outPort" : 2,
      "outVal" : 0,
      "setpoint" : 0
    },
    {
      "setpoint" : 50,
      "inPort" : 1,
      "outVal" : 1,
      "outPort" : 2,
      "inType" : "DIM",
      "outType" : "REL",
      "opr" : ">"
    },
    {
      "opr" : "<",
      "inType" : "DIM",
      "setpoint" : 50,
      "outVal" : 0,
      "inPort" : 1,
      "outPort" : 2,
      "outType" : "REL"
    },
    {
      "outPort" : 1,
      "setpoint" : 1,
      "inType" : "REL",
      "outType" : "REL",
      "inPort" : 1,
      "opr" : "=",
      "outVal" : 1
    },
    {
      "outPort" : 1,
      "inType" : "REL",
      "outType" : "REL",
      "outVal" : 1,
      "setpoint" : 1,
      "inPort" : 1,
      "opr" : "="
    },
    {
      "outType" : "REL",
      "inPort" : 1,
      "outPort" : 1,
      "inType" : "REL",
      "opr" : "=",
      "setpoint" : 1,
      "outVal" : 1
    },
    {
      "setpoint" : 1,
      "opr" : "=",
      "outType" : "REL",
      "inType" : "REL",
      "outPort" : 1,
      "inPort" : 1,
      "outVal" : 1
    },
    {
      "outType" : "REL",
      "opr" : "=",
      "inPort" : 1,
      "inType" : "REL",
      "outVal" : 1,
      "outPort" : 1,
      "setpoint" : 1
    },
    {
      "opr" : "=",
      "outPort" : 1,
      "inType" : "REL",
      "outVal" : 1,
      "setpoint" : 1,
      "inPort" : 1,
      "outType" : "REL"
    },
    {
      "outPort" : 1,
      "inPort" : 1,
      "setpoint" : 1,
      "outType" : "REL",
      "opr" : "=",
      "inType" : "REL",
      "outVal" : 1
    },
    {
      "outVal" : 1,
      "inPort" : 1,
      "inType" : "REL",
      "outType" : "REL",
      "setpoint" : 1,
      "outPort" : 1,
      "opr" : "="
    },
    {
      "opr" : "=",
      "setpoint" : 1,
      "outType" : "REL",
      "outVal" : 1,
      "outPort" : 1,
      "inPort" : 1,
      "inType" : "REL"
    },
    {
      "inPort" : 1,
      "outType" : "REL",
      "setpoint" : 1,
      "outVal" : 1,
      "inType" : "REL",
      "outPort" : 1,
      "opr" : "="
    },
    {
      "outVal" : 1,
      "inType" : "REL",
      "outType" : "REL",
      "outPort" : 1,
      "setpoint" : 1,
      "inPort" : 1,
      "opr" : "="
    },
    {
      "opr" : "=",
      "outPort" : 1,
      "outVal" : 1,
      "inPort" : 1,
      "outType" : "REL",
      "inType" : "REL",
      "setpoint" : 1
    }
  ]
}
)";
