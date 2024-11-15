#include <math.h>
#include <string>
#include <sstream>
#include <cstdlib>
#include <SPI.h>
#include "protocentral_Max30003.h" //MAX300003 ECG LIBRARY
// #define INT_PIN 02
// #include <esp_sleep.h>
int w = 0;
MAX30003 max30003;
#include "KickFiltersRT.h"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <NimBLE2904.h>
#include "mbedtls/aes.h" //LIBRARY FOR AES128 METHOD FOR ENCRYPTION
#include <esp_task_wdt.h>

// LIBRARY FOR APPLYING LOW PASS FILTERING FOR HR,RR,ECG
KickFiltersRT<signed long> filtersRT2;
KickFiltersRT<uint16_t> filtersRT3;
KickFiltersRT<uint16_t> filtersRT4;

// bool rtorIntrFlag = false;
// uint8_t statusReg[3];

uint64_t MACID = 0;
BLEServer *pServer = NULL;                                 // BLE SERVER
BLECharacteristic *pCharacteristic, *pCharacteristic_STAT; // BLE CHARACTERISTIC
NimBLEService *pService;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t count = 0;
signed long ECG_filter = 0;

uint16_t SQI = 0, RR_original = 0, RR_change;
uint32_t SUM_HR = 0, SUM_RR = 0;
uint32_t AvgHR_ALL_SAMPLE = 0, AvgRR_ALL_SAMPLE = 0;
uint16_t Heartrate, RRInterval;
int myValue;
uint8_t BLE_data[16], BLE_data1[16], BATTERY_PERC[1]; // BYTE ARRAY FOR SENDING DATA

int HR_SQI = 0, PERC_SQI = 0, REC_COUNT = 0;
uint16_t RRI_next, RRI_curr, RRI_prev = 0;
int RR_next_curr_perc, RR_curr_prec_perc;
int ii = 0;
int perc_HR = 0;
int LEAD_OFF_COUNT = 0;

#define ENCRYPTED_DATA_SIZE 16 // Adjust the size according to your data length

float analogvalue;

// Define the GPIO pin that is connected to the battery.
const int GPIO_BATTERY = 35;
// unsigned long previousMillis = 0;
// const long interval = 60000;             // 1 minutes in milliseconds
uint8_t previousBatteryPercentage = 100; // Initial battery percentage

int onetime_ECG = 0;
int RR_CONSTANT = 0;

uint32_t LEADOFF_COUNTER = 0, TOTAL_COUNTER = 0, LEADOFF_SEND_COUNTER = 0;
uint16_t battery_percentage, battery_percentage1;

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"             // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // rx characteristic UUID
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // tx characteristic UUID
#define CHARACTERISTIC_UUID_STAT "6E400006-B5A3-F393-E0A9-E50E24DCCA9E" // STAT notification characteristic UUID

// BLE DATA RECEIVE FUNCTION
class MyCallbacks : public BLECharacteristicCallbacks
{
private:
  int intValue;

public:
  // int intValue;
  void onWrite(BLECharacteristic *pRCharacteristic)
  {
    std::string rxValue = pRCharacteristic->getValue();

    if (rxValue.length() > 0)
    {

      intValue = std::stoi(rxValue);
      myValue = intValue;
    }
  }
  int getIntValue()
  {
    // Serial.println(intValue);
    return intValue;
  }
};

MyCallbacks myCallbacks;

// ONCONNECT AND ONDISCONNECT FUNCTION
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    // Serial.println(deviceConnected);
  }
};

// FUNCTION FOR RECONNECTION
void checkToReconnect() // added
{
  // disconnected so advertise
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500); // give the bluetooth stack the chance to get things ready
    // pServer->startAdvertising(); // restart advertising
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    oldDeviceConnected = deviceConnected;
  }
  // connected so reset boolean control
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }
}

// Function to encrypt data using AES CBC mode
void encryptBLEData(uint8_t *data, size_t dataLength, uint8_t *encryptedData)
{
  mbedtls_aes_context aes;
  unsigned char key[16] = "itzkbgulrcsjmnv"; // Example key
  key[15] = 'x';
  unsigned char iv[16] = "abcdefghijklmno"; // Example initialization vector
  iv[15] = 'p';

  // Set up the encryption context
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, key, 128);

  // Encrypt the data using AES in CBC mode
  mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, dataLength, iv, data, encryptedData);

  // Clean up the AES context
  mbedtls_aes_free(&aes);
}

// void rtorInterruptHndlr(){
//   rtorIntrFlag = true;
// }

// void enableInterruptPin(){

//   pinMode(INT_PIN, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(INT_PIN), rtorInterruptHndlr, CHANGE);
// }

void setup()
{
  Serial.begin(57600); // Serial begin
  // Serial.println("test");
  // delay(10000);

  filtersRT2.initmovingAverage(2); // for ECG
  filtersRT3.initmovingAverage(2); // for HR
  filtersRT4.initmovingAverage(2); // for RR

  pinMode(MAX30003_CS_PIN, OUTPUT);
  digitalWrite(MAX30003_CS_PIN, HIGH); // disable device
  pinMode(35, INPUT);
  // START OF SPI COMMUNICATION
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  // Serial.println("test1");

  bool ret = max30003.max30003ReadInfo();

  if (ret)
  {
    Serial.println("Max30003 read ID Success");
  }
  else
  {

    while (!ret)
    {
      // stay here untill the issue is fixed.
      ret = max30003.max30003ReadInfo();
      Serial.println("Failed to read ID, please make sure all the pins are connected");
      delay(5000);
    }
  }
  Serial.println("Initialising the chip ...");
  max30003.max30003Begin();
  // max30003.max30003BeginRtorMode();   // initialize MAX30003
  // enableInterruptPin();
  // max30003.max30003RegRead(STATUS, statusReg);
  MACID = ESP.getEfuseMac();

  // Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(MACID >> 32));  // Print High 2 bytes
  // Serial.printf("%08X\n", (uint32_t)MACID);                        // Print Low 4 bytes.

  // GENERATION OF DEVICE ID
  String myString = "TOP###" + String(MACID, HEX);
  const char *cString = myString.c_str();
  Serial.println(cString);

  // Create the BLE Device
  NimBLEDevice::init(cString); // Give it a name
  // Create the BLE Server
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE DATA TX Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);

  pCharacteristic->addDescriptor(new NimBLE2904());

  // Create a BLE STATASTIC TX Characteristic
  pCharacteristic_STAT = pService->createCharacteristic(
      CHARACTERISTIC_UUID_STAT,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);

  pCharacteristic_STAT->addDescriptor(new NimBLE2904());

  // Create a BLE DATA RX Characteristic
  BLECharacteristic *pRCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);

  // pCharacteristic->setCallbacks(new MyCallbacks());
  pRCharacteristic->setCallbacks(&myCallbacks);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  Serial.println("Waiting a client connection to notify...");
  // delay(2000);
  esp_task_wdt_init(1, true);
}

void loop()
{
  // checkToReconnect();
  // WAIT FOR THE DEVICE CONNECTED WITH APP
  while (deviceConnected)
  {

    while (w < 100)
    {
      max30003.getEcgSamples();
      ECG_filter = filtersRT2.movingAverage(max30003.ecgdata);
      Serial.println("For ECG reading stabilization");
      w++;
    }

    // delay(1000);
    bool indicate = false;
    myValue = myCallbacks.getIntValue();

    // Serial.println(myValue);
    // esp_task_wdt_reset(); // Reset watchdog timer
    while (myValue > 0 && deviceConnected == true)
    {
      esp_task_wdt_reset(); // Reset watchdog timer
      unsigned long t1 = micros();

      analogvalue = analogRead(GPIO_BATTERY);

      if (analogvalue > 3700)
      {
        analogvalue = 3700;
      }

      if (analogvalue < 2900)
      {
        analogvalue = 2900;
      }

      battery_percentage1 = map(analogvalue, 2900, 3700, 1, 100);

      if (battery_percentage1 <= previousBatteryPercentage)
      {
        previousBatteryPercentage = battery_percentage1;
        battery_percentage = battery_percentage1;
      }

      max30003.getEcgSamples(); // It reads the ecg sample and stores it to max30003.ecgdata .
      ECG_filter = filtersRT2.movingAverage(max30003.ecgdata);
      // Serial.println(ECG_filter);
      //  Serial.print(",");

      // if(rtorIntrFlag){

      // rtorIntrFlag = false;
      // max30003.max30003RegRead(STATUS, statusReg);

      // if(statusReg[1] & RTOR_INTR_MASK){
      // Serial.println("-------------------------------RTOR detected-------------------------->");
      max30003.getHRandRR();
      Heartrate = max30003.heartRate;
      RRInterval = max30003.RRinterval;
      //}
      //}

      count++;
      if (count == 4294967290)
      {
        count = 1;
      }

      myValue = myCallbacks.getIntValue();

      if (myValue == 2)
      {
        SUM_HR = SUM_HR + Heartrate;
        SUM_RR = SUM_RR + RRInterval;

        REC_COUNT++;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //                          value of Heartrate < 40 and > 150 will be counted and impacted SQI value                                       //
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (Heartrate < 40 || Heartrate > 180)
        {
          HR_SQI++;
        }

        RR_change = ((RRInterval - RR_original) < 0) ? (RRInterval - RR_original) : (RR_original - RRInterval);

        RR_original = RRInterval;

        if (RR_change > 0)
        {
          RR_CONSTANT++;

          ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          //                                         20% either exclusion method for SQI                                                            //
          ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
          RRI_next = RRI_curr;
          RRI_curr = RRI_prev;
          RRI_prev = RRInterval;

          if (RRI_curr and RRI_next != 0)
          {
            RR_next_curr_perc = int(abs(((float)(RRI_next - RRI_curr) / RRI_curr) * 100));
            RR_curr_prec_perc = int(abs(((float)(RRI_curr - RRI_prev) / RRI_prev) * 100));
          }

          if (RR_curr_prec_perc > 20 || RR_next_curr_perc > 20)
          {
            PERC_SQI++;
          }
        }
      }
      uint16_t LEADOFF = 0;

      // TOTAL_COUNTER++;
      if (ECG_filter > 0)
      {
        LEADOFF_COUNTER++;
      }
      else
      {
        LEADOFF_COUNTER = 0;
      }
      if(LEADOFF_COUNTER > 15)
      {
        LEADOFF_SEND_COUNTER++;
      }

      // int perc_LEADOFF = (float)LEADOFF_COUNTER / TOTAL_COUNTER * 100;
      // Serial.print(ECG_filter);
      // Serial.print(",");
      // Serial.print(perc_LEADOFF);
      // Serial.print(",");
      // if (perc_LEADOFF == 0)
      // {
      //   LEADOFF_SEND_COUNTER++;
      // }
      // else
      // {
      //   LEADOFF_SEND_COUNTER = 0;
      // }
      Serial.println(LEADOFF_SEND_COUNTER);
      // Serial.println(max30003.ecgdata);

      if (LEADOFF_SEND_COUNTER > 5)
      {
        LEADOFF_SEND_COUNTER = 0;
        LEADOFF = 1;
        LEADOFF_COUNTER = 0;
        // if (myValue = 2)
        // {
        //   LEAD_OFF_COUNT++;
        // }
      }
      // if (TOTAL_COUNTER > 1000)
      // {
      //   TOTAL_COUNTER = 0;
      //   LEADOFF_COUNTER = 0;
      // }

      //////////////////////////// Logic for ECG Stabilization /////////////////////////
      if (onetime_ECG == 0 && (Heartrate < 41 || Heartrate > 151))
      {
        ECG_filter = 0;
      }
      else
      {
        onetime_ECG = 1;
      }

      /////////////////////////////////////////////////////////////////////////////////////////

      BLE_data[0] = count;
      BLE_data[1] = count >> 8;
      BLE_data[2] = count >> 16;
      BLE_data[3] = count >> 24;
      BLE_data[4] = ECG_filter;
      BLE_data[5] = ECG_filter >> 8;
      BLE_data[6] = ECG_filter >> 16;
      BLE_data[7] = ECG_filter >> 24;
      BLE_data[8] = Heartrate;
      BLE_data[9] = Heartrate >> 8;
      BLE_data[10] = RRInterval;
      BLE_data[11] = RRInterval >> 8;
      BLE_data[12] = battery_percentage;
      BLE_data[13] = battery_percentage >> 8;
      BLE_data[14] = LEADOFF;
      BLE_data[15] = LEADOFF >> 8;

      // Serial.print(ECG_filter);
      // Serial.print(",");
      // Serial.print(Heartrate);
      // Serial.print(",");
      // Serial.print(RRInterval);
      // Serial.print(",");
      // Serial.print(battery_percentage);
      // Serial.print(",");
      // Serial.println(LEADOFF);

      size_t dataLength = sizeof(BLE_data);
      LEADOFF = 0;

      uint8_t encryptedData[dataLength];
      // Encrypt the data
      encryptBLEData(BLE_data, dataLength, encryptedData);

      pCharacteristic->setValue(encryptedData, 16);
      pCharacteristic->notify(true); // Send the value to the app!
      // Serial.print(xPortGetFreeHeapSize());
      // Serial.print(",");
      // Serial.println(xPortGetMinimumEverFreeHeapSize());
      // Serial.print(",");
      // Serial.println(deviceConnected);
      indicate = true;
      unsigned long t2 = micros();
      unsigned long t2t1 = 7812.5 - (t2 - t1);
      // Serial.println(t2t1);

      delayMicroseconds(t2t1);
    }
    esp_task_wdt_reset(); // Reset watchdog timer
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    if (indicate == true)
    {
      if (REC_COUNT != 0)
      {
        perc_HR = (float)HR_SQI / REC_COUNT * 100;
      }

      // Serial.println(RR_CONSTANT);
      // Serial.println(PERC_SQI);
      // Serial.println(LEAD_OFF_COUNT);
      // Serial.println(perc_HR);

      if (RR_CONSTANT > 10 && LEAD_OFF_COUNT < 2)
      {
        if (perc_HR < 21)
        {
          if (PERC_SQI < 21)
          {
            SQI = 5;
            Serial.println("Good");
          }
          else
          {
            SQI = 10;
            Serial.println("Normal");
          }
        }
        else if (20 < perc_HR < 41)
        {
          SQI = 10;
          Serial.println("Normal");
        }
        else
        {
          SQI = 15;
          Serial.println("Bad 1");
        }
      }
      else
      {
        SQI = 15;
        Serial.println("Bad 2");
      }
      /////////////////////////////////////////////////////////////////////////////////////////////////

      // myValue = 0;
      ECG_filter = 0;
      Heartrate = 0;
      RRInterval = 0;
      // delay(500);

      if (REC_COUNT != 0)
      {
        AvgHR_ALL_SAMPLE = int(SUM_HR / REC_COUNT);
        AvgRR_ALL_SAMPLE = int(SUM_RR / REC_COUNT);
      }

      BLE_data1[0] = SQI;
      BLE_data1[1] = SQI >> 8;
      BLE_data1[2] = SQI >> 16;
      BLE_data1[3] = SQI >> 24;
      BLE_data1[4] = AvgHR_ALL_SAMPLE;
      BLE_data1[5] = AvgHR_ALL_SAMPLE >> 8;
      BLE_data1[6] = AvgHR_ALL_SAMPLE >> 16;
      BLE_data1[7] = AvgHR_ALL_SAMPLE >> 24;
      BLE_data1[8] = AvgRR_ALL_SAMPLE;
      BLE_data1[9] = AvgRR_ALL_SAMPLE >> 8;
      BLE_data1[10] = AvgRR_ALL_SAMPLE >> 16;
      BLE_data1[11] = AvgRR_ALL_SAMPLE >> 24;
      BLE_data1[12] = PERC_SQI;
      BLE_data1[13] = PERC_SQI >> 8;
      BLE_data1[14] = perc_HR;
      BLE_data1[15] = perc_HR >> 8;

      size_t dataLength1 = sizeof(BLE_data1);
      // Allocate memory for encrypted data
      uint8_t encryptedData1[dataLength1];
      // Encrypt the data
      encryptBLEData(BLE_data1, dataLength1, encryptedData1);

      Serial.println("Statistics:");
      for (int i = 0; i < dataLength1; i++)
      {
        Serial.print(BLE_data1[i], HEX);
        Serial.print(' '); // Add a space for readability
      }
      Serial.println(); // Print a newline at the end

      while (deviceConnected)
      {
        pCharacteristic_STAT->setValue(encryptedData1, 16);
        pCharacteristic_STAT->notify(); // Send the value to the app!
      }

      SQI = 0;
      AvgHR_ALL_SAMPLE = 0;
      AvgRR_ALL_SAMPLE = 0;
      w = 0;
      indicate = false;
      delay(500);
      ESP.restart();
    }
  }
}