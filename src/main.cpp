
#include "main.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

portMUX_TYPE ecgMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ecgSamplesMux = portMUX_INITIALIZER_UNLOCKED;
//portENTER_CRITICAL_ISR(&ecgSamplesMux);
//portEXIT_CRITICAL_ISR(&ecgSamplesMux);

volatile struct mainPackStructure mainPackStructure = { .res = ACK, .packType = 1, .batteryPercent = 0, .ecgPartsCount = 0, .loP = 0, .loN = 0};
struct errortPacksStructure errortPacksStructure = { .res = NAK, .packType = 2, .requestPackType = 0, .errorCode = 0};

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t __rowNum = 1;

// These constants won't change:
const int batterySensorPin = 32;  // 32 pin
const int vccAdcPin = 34;  // 32 pin

// ECG
const int ecgSensorPin = 35;  // 35 pin
const int loNegativePin = 25;
const int loPositivePin = 26;
const int ecgSdnPin = 27;

// variables:
volatile uint8_t battPercent = 0;
volatile float calcVoltage = 0;
volatile uint16_t sensorValue = 0;   // the sensor value
#define SnakeRollLen 10
int32_t snakeRoll[SnakeRollLen]; 
uint8_t currentSnakeRollPosition = 0;
bool _isFullSnakeRoll = false;

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c3319333"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
//

// VIBRO
uint8_t virboPin = 23;

// Sleep
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
RTC_DATA_ATTR int bootCount = 0;

const int _cyclesBeforeSleep = 20;
int _currentCycleBeforeSleep = 0;

// TIMERS
hw_timer_t *Timer0_Cfg = NULL;
bool measurementEcgInProgress = false;
volatile SemaphoreHandle_t timer0Semaphore;
portMUX_TYPE timer0Mux = portMUX_INITIALIZER_UNLOCKED;

// ECG Measure
uint16_t ecgSamples[ECG_LEN];
uint16_t currentEcgSampleIdx = 0;

bool haveNewEcgMeasure = false;

// for transmit ecg data with BLE
#define FrameSize 128
volatile bool transmitted = false;
 

void sendStructure() {
  // сразу отправим и состояние присосок ЭКГ
  calcBattValues();
  portENTER_CRITICAL_ISR(&ecgMux);
  mainPackStructure.batteryPercent = battPercent;
  mainPackStructure.loN = digitalRead(loNegativePin);
  mainPackStructure.loP = digitalRead(loPositivePin);

  pCharacteristic->setValue((uint8_t*)&mainPackStructure, sizeof(mainPackStructure));
  pCharacteristic->notify();

  portEXIT_CRITICAL_ISR(&ecgMux);
}

void goToSleep() {
  display.clearDisplay();
  display.display();
  delay(100);
  esp_deep_sleep_start();
}

// вывод причины пробуждения ESP. для тестов
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
};

void IRAM_ATTR Timer0_ISR()
{
  portENTER_CRITICAL_ISR(&ecgSamplesMux);
  ecgSamples[currentEcgSampleIdx] = analogRead(ecgSensorPin);
  portEXIT_CRITICAL_ISR(&ecgSamplesMux);

  if (currentEcgSampleIdx++ >= ECG_LEN) { // Хватит)
    timerAlarmDisable(Timer0_Cfg);
    measurementEcgInProgress = false;
    haveNewEcgMeasure = true;
  }
   
};

void setAndSendError(std::string s, uint8_t errorCode, uint8_t requestPackType) {
  const char* str = s.c_str();
  memset(&errortPacksStructure.errorData, 0, sizeof(errortPacksStructure.errorData));
  strncpy(errortPacksStructure.errorData,str,strlen(str));
  errortPacksStructure.errorCode = errorCode;
  errortPacksStructure.requestPackType = requestPackType;

  pCharacteristic->setValue((uint8_t*)&errortPacksStructure, sizeof(errortPacksStructure));
  pCharacteristic->notify();
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      display.println("BLE dev connected");
      display.display();
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      display.println("BLE dev disconn");
      display.display();
      deviceConnected = false;
    };
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0) {
      // напечатаем то, что прилетело в характеристику
      Serial.println("****ble incame*****");
      for (int i = 0; i < value.length(); i++)
        Serial.print(value[i]);
      Serial.println();

      // дальше обработка команды
      uint8_t* bytes = pCharacteristic->getData();
      if (value == "ping") { /// это просто пинг, на который должно вернуться значение эелементов структуры
        Serial.println("icame pack type ping");
        sendStructure();
      } 
      else if (value == "makeecg") { /// 
        Serial.println("icame pack type makeecg");
        // if (digitalRead(loPositivePin) == HIGH && digitalRead(loNegativePin) == HIGH) {
        //   setAndSendError("LO+ & LO- are in HIGH", 12, 2);
        //   Serial.println("--> LO+ & LO- are in HIGH");
        //   return;
        // }
        returnAckResponse();
        getEcgADC();
      } 
      else if (bytes[0] == 0x55) { /// чтение куска
        int frameNumber = bytes[1];
        Serial.println("icame read frame pack type makeecg");
        ecgResponsePacksStructure st = { .res = ACK };
        st.ecgvals = (ecgSamples + (ECG_LEN / FrameSize * frameNumber));
        pCharacteristic->setValue((uint8_t*)&st, (ECG_LEN / FrameSize * 2) + 1);
        pCharacteristic->notify();
        Serial.println("pack sent");  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      }
      else if (bytes[0] == 0x56) { /// отправить спать
        uint16_t seconds = 0;
        memcpy(&seconds, bytes + 1, 2);

        Serial.print("icame SLEEP pack sec ");
        Serial.println(seconds);
        esp_sleep_enable_timer_wakeup(seconds * uS_TO_S_FACTOR);
        Serial.flush(); 
        goToSleep();
      }
    }
  }
};

// Получение данных ЭКГ
void getEcgADC() {
  Serial.println("Start measure ECG...");
  currentEcgSampleIdx = 0;
  haveNewEcgMeasure = false;
  measurementEcgInProgress = true;
  //timerAlarmEnable(Timer0_Cfg);  
  unsigned long mi = millis();
  for (int i = 0; i < ECG_LEN; i++)
  {
    ecgSamples[currentEcgSampleIdx] = analogRead(ecgSensorPin);
    delay(1);
  }
  Serial.print ("ECG Measure time - ");
  Serial.println(millis() - mi);
  measurementEcgInProgress = false;
  haveNewEcgMeasure = true;
};

void clearDisplayExceptBattinfo() {
  display.clearDisplay();
  display.setCursor(85,0);
  display.setTextSize(1); 
  //display.setTextSize(2); 

  display.print(calcVoltage);

  

  display.print("%");
  display.setTextSize(1);  
  display.setCursor(0,0); 
  display.display();
};

void updateScreen() {
  display.clearDisplay();
  display.setTextSize(1); 
  //LO
  display.setCursor(0,0);
  display.print("LO+"); 
  display.setCursor(20,0);
  display.print(digitalRead(loPositivePin));  
  display.setCursor(30,0);
  display.print("LO-"); 
  display.setCursor(50,0);
  display.print(digitalRead(loNegativePin));
  display.setCursor(60,0);
  display.print("ecgval"); 
  display.setCursor(100,0);
  uint16_t ecgval = readAvgAdc(ecgSensorPin, 5);
  display.print(ecgval);

  // BATT
  display.setCursor(0,20);
  display.print(sensorValue);
  display.setCursor(30,20);
  display.print((int)calcVoltage);
  display.setCursor(60,20);
  display.print(battPercent);

  //vcc
  display.setCursor(0,40);
  uint16_t vccadcval = readAvgAdc(vccAdcPin, 6);
  display.print(vccadcval);
  display.display();
}

void setupDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  Serial.println("OLED begun");
  display.display();
}

void setup() {

  Serial.begin(115200);

  //test
  portENTER_CRITICAL_ISR(&ecgSamplesMux);
  for(int i = 0; i< ECG_LEN; i++) {
    ecgSamples[i] = i;
  }  
  portEXIT_CRITICAL_ISR(&ecgSamplesMux);


  //

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  // lets vibrate
  pinMode(virboPin, OUTPUT);
  digitalWrite(virboPin, HIGH);
  delay(200);
  digitalWrite(virboPin, LOW);


  setupDisplay();
  display.println("iNIT BEGIN...");
  display.display();

  // Create the BLE Device
  BLEDevice::init("CardIQ");
  display.println("BLEDevice init");
  display.display();
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->setCallbacks(new MyCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  // Start the service
  pService->start();

   // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("ble 0k");
  Serial.println("Waiting a client connection to notify...");

  // TIMER SETUP
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 2500, true);

  Serial.println("timers 0k");

  // ECG pins
  pinMode(loPositivePin, INPUT);
  pinMode(loNegativePin, INPUT);
  pinMode(ecgSdnPin, OUTPUT);
  digitalWrite(ecgSdnPin, HIGH);

  display.println("iNIT OK");
  display.display();
};

void calcBattValues() {
  // BATT INFO:
  sensorValue = readAvgAdc(batterySensorPin, 6);
  calcVoltage = sensorValue * 3492.0 / 4095.0 * 1.47;
  battPercent = 100 - ((4200 - calcVoltage ) * 100 / (4200-3200));
}

// Здесь отображаем информацию по батарейке
void getAndShowBatteryInfo() {
  portENTER_CRITICAL_ISR(&ecgMux);
  mainPackStructure.batteryPercent = battPercent;
  portEXIT_CRITICAL_ISR(&ecgMux);
  //clearDisplayExceptBattinfo();
  updateScreen();
}

void loop() {

  // по окончании чтения данных ECG
  // отправим их подключенному BLE устройству

  if (haveNewEcgMeasure && !measurementEcgInProgress) {
    
    Serial.print("Measure ECG comlited. From loop. Time ");
    portENTER_CRITICAL_ISR(&ecgMux);
    mainPackStructure.ecgPartsCount = ECG_LEN / FrameSize;
    portEXIT_CRITICAL_ISR(&ecgMux);
    haveNewEcgMeasure = false;
  }

  // BLE Section
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("disconnected. start advertising again");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  delay(500);
};

// Чтение усредненного значения ADC
uint16_t readAvgAdc(uint8_t pin, uint8_t avgAdcSampling) {
  long mils = millis();
  int readings = 0;
  for (int i = 0; i < avgAdcSampling; i++)
  {
      readings += analogRead(pin);
  }
  // Serial.print("read adc time is (ms) - ");
  // Serial.println(millis() - mils);
  return readings / avgAdcSampling;
}

void returnAckResponse() {
  pCharacteristic->setValue((uint8_t*)&ackResponse, 1);
  pCharacteristic->notify();  
}
