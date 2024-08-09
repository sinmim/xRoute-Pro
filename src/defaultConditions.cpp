const char *defaultCondition = R"(
{
  "conditions": [
    {
      "inType": "FLT",
      "inPrt": 0,
      "oprt": ">",
      "setPnt": 800,
      "outType": "DIM",
      "outputPort": 1,
      "outputValue": 0
    },
    {
      "inType": "FLT",
      "inPrt": 0,
      "oprt": "<",
      "setPnt": 800,
      "outType": "DIM",
      "outputPort": 1,
      "outputValue": 100
    },
    {
      "inType": "REL",
      "inPrt": 1,
      "oprt": "==",
      "setPnt": 1,
      "outType": "DIM",
      "outputPort": 2,
      "outputValue": 100
    },
    {
      "inType": "REL",
      "inPrt": 1,
      "oprt": "==",
      "setPnt": 0,
      "outType": "DIM",
      "outputPort": 2,
      "outputValue": 0
    },
    {
      "inType": "AMP",
      "inPrt": 2,
      "oprt": ">",
      "setPnt": 20,
      "outType": "DIM",
      "outputPort": 3,
      "outputValue": 100
    },
    {
      "inType": "AMP",
      "inPrt": 2,
      "oprt": "<",
      "setPnt": 20,
      "outType": "DIM",
      "outputPort": 3,
      "outputValue": 0
    }
  ]
}
)";
