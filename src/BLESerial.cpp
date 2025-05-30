/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.
[E][BluetoothSerial.cpp:181] _spp_send_buffer(): SPP Write Congested!
   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second.
*/
/*------------------Passkey and sequrity by
  https://github.com/0015/ThatProject/blob/master/ESP32_TTGO/BLE_Secure_Server/BLE_Secure_Server.ino
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include "relay.h"
#include "BLESerial.h"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bleData BLE_DATA;

#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"             //"6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RXTX "0000ffe1-0000-1000-8000-00805f9b34fb" //"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

uint32_t PASSKEY;
// #define PASSKEY 123456

void remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();

    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    if (!dev_list)
    {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc failed, return\n");
        return;
    }
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++)
    {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }

    free(dev_list);
}

void bleSecurity()
{
    // esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND; // ESP_LE_AUTH_REQ_SC_MITM_BOND; // ESP_LE_AUTH_REQ_SC_BOND;//by saman it saves authorized devices and dont delet them weith reset

    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = PASSKEY;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
}

void bleSetPass(uint32_t pass)
{
    PASSKEY = pass;
    bleSecurity();
}

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {

        BLE_DATA.deviceConnected = true;
        // pServer->getAdvertising()->start();
        // bleSecurity();
    };

    void onDisconnect(BLEServer *pServer)
    {
        BLE_DATA.deviceConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string strBLE = pCharacteristic->getValue();
        // Serial.print("BLE:");
        for (int i = 0; i < strBLE.length(); i++)
        {
            // Serial.print(strBLE[i]);          // print received data to terminal
            BLE_DATA.bleRxStr[i] = strBLE[i]; // fill data buffer
        }
        // BLE_DATA.bleRxStr[strBLE.length() - 1] = '\0'; // delet the last buffer to avoid \n character
        BLE_DATA.bleRxStr[strBLE.length()] = '\0'; // new change for multi command in a single message : 23/5/20204
        BLE_DATA.RxDataReadyFlag = true;
    }
    void onNotify(BLECharacteristic *pCharacteristic)
    {
    }
};

class SecurityCallback : public BLESecurityCallbacks
{

    uint32_t onPassKeyRequest()
    {
        return 000000;
    }

    void onPassKeyNotify(uint32_t pass_key) {}

    bool onConfirmPIN(uint32_t pass_key)
    {
        Serial.println("AliPass" + String(pass_key));
        vTaskDelay(5000);
        return true;
    }

    bool onSecurityRequest()
    {
        return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl)
    {
        int numBondedDevices = esp_ble_get_bond_device_num();
        Serial.println("ConnId:" + String(pServer->getConnId()));
        Serial.println("ConnectedCount:" + String(pServer->getConnectedCount()));
        Serial.println("bd_addr:" + String(cmpl.bd_addr[0]) + ":" + String(cmpl.bd_addr[1]) + ":" + String(cmpl.bd_addr[2]) + ":" + String(cmpl.bd_addr[3]));
        Serial.println("faileReason" + String(cmpl.fail_reason));

        if (cmpl.success)
        {
            Serial.println("   - SecurityCallback - Authentication Success");
        }
        else
        {
            Serial.println("   - SecurityCallback - Authentication Failure*");
            pServer->removePeerDevice(pServer->getConnId(), true);
        }
        BLEDevice::startAdvertising(); /// by saman for multiple connections
    }
    /*
         void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl)
    {
        int numBondedDevices = esp_ble_get_bond_device_num();
        Serial.println("ConnId:" + String(pServer->getConnId()));
        Serial.println("ConnectedCount:" + String(pServer->getConnectedCount()));
        Serial.print("bd_addr: ");
        for (int i = 0; i < 6; i++)
        {
            Serial.print(String(cmpl.bd_addr[i], HEX) + (i < 5 ? ":" : "\n"));
        }
        Serial.print("failReason: ");
        Serial.println(String(cmpl.fail_reason));

        if (cmpl.success)
        {
            Serial.println("   - SecurityCallback - Authentication Success");
        }
        else
        {
            Serial.println("   - SecurityCallback - Authentication Failure*");
            switch (cmpl.fail_reason)
            {
                case ESP_AUTH_SMP_PASSKEY_FAIL:
                    Serial.println("Reason: The user input of passkey failed");
                    break;
                case ESP_AUTH_SMP_OOB_FAIL:
                    Serial.println("Reason: The OOB data is not available");
                    break;
                case ESP_AUTH_SMP_PAIR_AUTH_FAIL:
                    Serial.println("Reason: The authentication requirements cannot be met");
                    break;
                case ESP_AUTH_SMP_CONFIRM_VALUE_FAIL:
                    Serial.println("Reason: The confirm value does not match the calculated comparison value");
                    break;
                case ESP_AUTH_SMP_PAIR_NOT_SUPPORT:
                    Serial.println("Reason: Pairing is not supported by the device");
                    break;
                case ESP_AUTH_SMP_ENC_KEY_SIZE:
                    Serial.println("Reason: The resultant encryption key size is not long enough");
                    break;
                case ESP_AUTH_SMP_INVALID_CMD:
                    Serial.println("Reason: The SMP command received is not supported by this device");
                    break;
                case ESP_AUTH_SMP_UNKNOWN_ERR:
                    Serial.println("Reason: Pairing failed due to an unspecified reason");
                    break;
                case ESP_AUTH_SMP_REPEATED_ATTEMPT:
                    Serial.println("Reason: Pairing or authentication procedure is disallowed");
                    break;
                case ESP_AUTH_SMP_INVALID_PARAMETERS:
                    Serial.println("Reason: The command length is invalid or a parameter is outside the specified range");
                    break;
                case ESP_AUTH_SMP_DHKEY_CHK_FAIL:
                    Serial.println("Reason: The DHKey Check value received doesn’t match the one calculated by the local device");
                    break;
                case ESP_AUTH_SMP_NUM_COMP_FAIL:
                    Serial.println("Reason: The confirm values in the numeric comparison protocol do not match");
                    break;
                case ESP_AUTH_SMP_BR_PARING_IN_PROGR:
                    Serial.println("Reason: Pairing Request sent over the BR/EDR transport is in progress");
                    break;
                case ESP_AUTH_SMP_XTRANS_DERIVE_NOT_ALLOW:
                    Serial.println("Reason: The BR/EDR Link Key or BLE LTK cannot be used to derive");
                    break;
                case ESP_AUTH_SMP_INTERNAL_ERR:
                    Serial.println("Reason: Internal error in pairing procedure");
                    break;
                case ESP_AUTH_SMP_UNKNOWN_IO:
                    Serial.println("Reason: Unknown IO capability, unable to decide association model");
                    break;
                case ESP_AUTH_SMP_INIT_FAIL:
                    Serial.println("Reason: SMP pairing initiation failed");
                    break;
                case ESP_AUTH_SMP_CONFIRM_FAIL:
                    Serial.println("Reason: The confirm value does not match");
                    break;
                case ESP_AUTH_SMP_BUSY:
                    Serial.println("Reason: Pending security request ongoing");
                    break;
                case ESP_AUTH_SMP_ENC_FAIL:
                    Serial.println("Reason: The Controller failed to start encryption");
                    break;
                case ESP_AUTH_SMP_STARTED:
                    Serial.println("Reason: SMP pairing process started");
                    break;
                case ESP_AUTH_SMP_RSP_TIMEOUT:
                    Serial.println("Reason: Security Manager timeout due to no SMP command being received");
                    break;
                case ESP_AUTH_SMP_DIV_NOT_AVAIL:
                    Serial.println("Reason: Encrypted Diversifier value not available");
                    break;
                case ESP_AUTH_SMP_UNSPEC_ERR:
                    Serial.println("Reason: Unspecified failed reason");
                    break;
                case ESP_AUTH_SMP_CONN_TOUT:
                    Serial.println("Reason: Pairing process failed due to connection timeout");
                    break;
                default:
                    Serial.println("Reason: Unknown");
                    break;
            }
        }
        BLEDevice::startAdvertising(); // for multiple connections
    }

    */
};

void setupBLE()
{

    BLEDevice::init("LabobinxSmart"); // Create the BLE Device
    BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
    BLEDevice::setSecurityCallbacks(new SecurityCallback());

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RXTX,
        BLECharacteristic::PROPERTY_WRITE); // PROPERTY_WRITE_NR cause write characteristics error in ios
    pTxCharacteristic->setIndicateProperty(true);
    pTxCharacteristic->setNotifyProperty(true);
    pTxCharacteristic->setReadProperty(true);
    pTxCharacteristic->addDescriptor(new BLE2902());
    pTxCharacteristic->setCallbacks(new MyCallbacks());

    pTxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED | ESP_GATT_PERM_WRITE_ENCRYPTED);
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);
    BLEDevice::startAdvertising();
    bleSecurity();

    Serial.println("BLE : Waiting a client connection to notify...");
}

void BLEsend(char *str)
{
    BLE_DATA.TxDataSent = false;
    if (BLE_DATA.deviceConnected)
    {
        // pTxCharacteristic->setValue(&txValue, 1);
        pTxCharacteristic->setValue(str);
        pTxCharacteristic->notify();
        BLE_DATA.txValue++;
        // delay(10); // bluetooth stack will go into congestion, if too many packets are sent
        vTaskDelay(15 / portTICK_PERIOD_MS);
    }
    BLE_DATA.TxDataSent = true;
}

#define MAX_CHUNK_SIZE 20

void bleSendLongString(String longString)
{
    int length = longString.length();
    int startIndex = 0;

    while (startIndex < length)
    {
        int chunkSize = min(MAX_CHUNK_SIZE, length - startIndex);
        String chunk = longString.substring(startIndex, startIndex + chunkSize);
        BLEsend((char *)chunk.c_str());
        startIndex += chunkSize;
    }
}

void BLEloop()
{

    if (BLE_DATA.deviceConnected)
    {
        if (strlen(BLE_DATA.bleTxStr))
        {
            BLEsend(BLE_DATA.bleTxStr);
            /*clear string after sending*/
            BLE_DATA.bleTxStr[0] = '\0';
        }
    }

    // disconnecting
    if (!BLE_DATA.deviceConnected && BLE_DATA.oldDeviceConnected)
    {
        // delay(500);                  // give the bluetooth stack the chance to get things ready
        vTaskDelay(500 / portTICK_PERIOD_MS);
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        BLE_DATA.oldDeviceConnected = BLE_DATA.deviceConnected;
    }
    // connecting
    if (BLE_DATA.deviceConnected && !BLE_DATA.oldDeviceConnected)
    {
        // do stuff here on connecting
        BLE_DATA.oldDeviceConnected = BLE_DATA.deviceConnected;
    }
}