#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t __rowNum = 1;

// These constants won't change:
const int batterySensorPin = A4;  // 32 pin
const uint8_t _probesToAvgADC = 30;


// variables:
uint8_t battPercent = 0;
int sensorValue = 0;   // the sensor value
#define SnakeRollLen 10
int32_t snakeRoll[SnakeRollLen]; 
uint8_t currentSnakeRollPosition = 0;
bool _isFullSnakeRoll = false;

// BLE
#define BatteryService BLEUUID((uint16_t)0x180F)
BLECharacteristic BatteryLevelCharacteristic(BLEUUID((uint16_t)0x2A19), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor BatteryLevelDescriptor(BLEUUID((uint16_t)0x2901));

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
hw_timer_t *Timer1_Cfg = NULL;
bool measurementEcgInProgress = false;
volatile SemaphoreHandle_t timer0Semaphore;
volatile SemaphoreHandle_t timer1Semaphore;
portMUX_TYPE timer0Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer1Mux = portMUX_INITIALIZER_UNLOCKED;


// ECG Measure
#define ECG_LEN 3200
uint16_t ecgSamples[ECG_LEN];
uint16_t currentEcgSampleIdx = 0;
const int ecgPin = A4;  // 32 pin  TEST this is Battery 4 testing
bool haveNewEcgMeasure = false;
uint64_t millisStart = 0;

// for transmit ecg data with BLE
#define FrameSize 64
#define FrameSizeToSend FrameSize*4
volatile bool transmitted = false;
 
void transmitEcgData() {
  Serial.println("Begin send ECG...");
  display.println("Begin send ECG...");
  display.display();
  for(int i = 0; i < ECG_LEN / FrameSize; i++) {
    Serial.println(i);
    pCharacteristic->setValue((uint8_t*)&ecgSamples + i * FrameSizeToSend, FrameSizeToSend);
    pCharacteristic->notify();
    delay(5);
  }
}

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

void IRAM_ATTR Timer1_ISR()
{
  xSemaphoreGiveFromISR(timer1Semaphore, NULL);
};

void IRAM_ATTR Timer0_ISR()
{
  ecgSamples[currentEcgSampleIdx] = analogRead(ecgPin);

  if (currentEcgSampleIdx++ >= ECG_LEN) { // Хватит)
    currentEcgSampleIdx = 0;
    measurementEcgInProgress = false;
    haveNewEcgMeasure = true;
  }
   
};

void getEcgADC() {
  Serial.println("Start measure ECG...");
  display.println("Start measure ECG");
  display.display();
  currentEcgSampleIdx = 0;
  haveNewEcgMeasure = false;
  measurementEcgInProgress = true;
  timerAlarmEnable(Timer0_Cfg);  
  millisStart = millis();
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      display.println("BLE dev connected");
      display.display();
      getEcgADC();
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      display.println("BLE dev disconn");
      display.display();
      deviceConnected = false;
    }
};

void clearDisplayExceptBattinfo() {
  display.clearDisplay();
  display.setCursor(85,0);
  display.setTextSize(2); 
  display.print(battPercent);
  display.print("%");
  display.setTextSize(1);  
  display.setCursor(0,0); 
  display.display();
};

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
  delay(1000);
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
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());
  // Start the service
  pService->start();

  // battery
  BLEService *pBattery = pServer->createService(BatteryService);
  pBattery->addCharacteristic(&BatteryLevelCharacteristic);
  BatteryLevelDescriptor.setValue("Percentage 0 - 100");
  BatteryLevelCharacteristic.addDescriptor(&BatteryLevelDescriptor);
  BatteryLevelCharacteristic.addDescriptor(new BLE2902());
  
  pServer->getAdvertising()->addServiceUUID(BatteryService);
  pBattery->start();

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
  // Going sleep timer
  timer1Semaphore = xSemaphoreCreateBinary();
  Timer1_Cfg = timerBegin(1, 8000, true);  // 10000Hz  0.0001s
  timerAttachInterrupt(Timer1_Cfg, &Timer1_ISR, true);
  timerAlarmWrite(Timer1_Cfg, 10000 * 60, true); // 1sec * 60 = 60sec
  timerAlarmEnable(Timer1_Cfg); // start timer1

  Serial.println("timers 0k");



  display.println("iNIT OK");
  display.display();
};

// Здесь отображаем информацию по батарейке
void getAndShowBatteryInfo() {
  // BATT INFO:
  sensorValue = analogRead(batterySensorPin);
  float calcVoltage = (float)sensorValue * 3390 / 4095 * 1.0182904068  * 1.47;
  battPercent = 100 - ((4200 - calcVoltage) * 100 / (4200-3200));
  clearDisplayExceptBattinfo();
}

void loop() {

  // по окончании чтения данных ECG
  // отправим их подключенному BLE устройству
  if (haveNewEcgMeasure && !measurementEcgInProgress) {
    timerAlarmDisable(Timer0_Cfg);
    Serial.print("Measure ECG comlited. From loop. Time ");
    Serial.println((millis() - millisStart));
    if (deviceConnected && !transmitted) {
      transmitted = true;
      haveNewEcgMeasure = false;
      transmitEcgData();

      //getEcgADC();
    }
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
      BatteryLevelCharacteristic.setValue(&battPercent, 1);
      BatteryLevelCharacteristic.notify();   
      oldDeviceConnected = deviceConnected;
  }

  // Здесь отображаем информацию по батарейке
  getAndShowBatteryInfo();
  delay(500);
};