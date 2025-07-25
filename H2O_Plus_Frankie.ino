#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <REG.h>
#include <wit_c_sdk.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>

#define SDA_PIN 9   
#define SCL_PIN 10  
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE 0x08
#define READ_UPDATE 0x80
#define SERVICE_UUID "9d48fca6-13a8-4090-8d08-2598d2ad7d1b"
#define CHARACTERISTIC_UUID "9d48fca7-13a8-4090-8d08-2598d2ad7d1b"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

static volatile char s_cDataUpdate = 0; 
const uint32_t c_uiBaud[7] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
bool deviceConnected = false;

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    delay(500);                  
    pServer->startAdvertising();  
    deviceConnected = true;
  };
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

void setup() {

  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  aht.begin();
  bmp.begin(0x77);

  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
  AutoScanSensor();

  BLEDevice::init("H20_Plus_Frankie");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE
  );
  pCharacteristic->addDescriptor(new BLE2902());
  descriptor_2901 = new BLE2901();
  descriptor_2901->setDescription("Cabin Data.");
  descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ);
  pCharacteristic->addDescriptor(descriptor_2901);
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); 
  BLEDevice::startAdvertising();
}

void loop() {

  sensors_event_t humidity, temperature;
  aht.getEvent(&humidity, &temperature);

  int16_t t = 0;
  int16_t p = 0;
  int16_t h = 0;
  int16_t x = 0;
  int16_t y = 0;

  t = (bmp.readTemperature()-6.0)*10.0;
  p = (bmp.readPressure()/10.0)*1.0;
  h = (humidity.relative_humidity)*10.0;

  while (Serial1.available()) WitSerialDataIn(Serial1.read());

  if (s_cDataUpdate & ANGLE_UPDATE) {
    x = (sReg[Roll] / 32768.0f * 180.0f)*100.0;
    y = (sReg[Pitch] / 32768.0f * 180.0f)*100.0;
    s_cDataUpdate = 0;

    if (deviceConnected) {
      uint8_t k[10];
      k[0] = highByte(x);
      k[1] = lowByte(x);
      k[2] = highByte(y);
      k[3] = lowByte(y);
      k[4] = highByte(t);
      k[5] = lowByte(t);
      k[6] = highByte(h);
      k[7] = lowByte(h);
      k[8] = highByte(p);
      k[9] = lowByte(p);
      pCharacteristic->setValue(k, sizeof(k));
      pCharacteristic->notify();
    }

  }

  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(t);
  Serial.print(", ");
  Serial.print(h);
  Serial.print(", ");
  Serial.print(p);
  Serial.println("");

  delay(500);

}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  Serial1.write(p_data, uiSize);
  Serial1.flush();
}

static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for (i = 0; i < uiRegNum; i++) {
    switch (uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

static void AutoScanSensor(void) {
  int i, iRetry;
  for (i = 0; i < sizeof(c_uiBaud) / sizeof(c_uiBaud[0]); i++) {
    Serial1.begin(c_uiBaud[i], SERIAL_8N1, 20, 21);
    Serial1.flush();
    iRetry = 2;
    s_cDataUpdate = 0;
    do {
      WitReadReg(AX, 3);
      delay(200);
      while (Serial1.available()) WitSerialDataIn(Serial1.read());
      if (s_cDataUpdate != 0) {
        return;
      }
      iRetry--;
    } while (iRetry);
    
  }

}
