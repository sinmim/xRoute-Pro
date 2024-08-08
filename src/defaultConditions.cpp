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
    }
  ]
}
)";
