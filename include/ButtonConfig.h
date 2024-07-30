const char *defaultConfig = R"({
        "buttons": [
          {
            "name": "Toilet",
            "port": 1,
            "image": 0
          },
          {
            "name": "refrigerator",
            "port": 1,
            "image": 1
          },
          {
            "name": "waterpump",
            "port": 2,
            "image": 2
          },
          {
            "name": "airheater",
            "port": 3,
            "image": 4
          },
          {
            "name": "waterheater",
            "port": 5,
            "image": 5
          },
          {
            "name": "usb",
            "port": 6,
            "image": 7
          },
          {
            "name": "inverter",
            "port": 8,
            "image": 9
          }
        ],
        "dimmers": [
          {
            "name": "Kitchen",
            "port": 1
          },
          {
            "name": "Coridor",
            "port": 2
          },
          {
            "name": "Bed",
            "port": 3
          },
          {
            "name": "Outside",
            "port": 4
          },
          {
            "name": "Front",
            "port": 5
          }
        ],
        "rgb": {
          "name": "RGB1",
          "ports": {
            "RED": 1,
            "GREEN": 2,
            "BLUE": 3
          }
        }
    })";