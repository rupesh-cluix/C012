#ifndef XPOWERS_NO_ERROR
// #error "Running this example is known to not damage the device! Please go and uncomment this!"
#endif
// Defined using AXP2102
//#define XPOWERS_CHIP_AXP2101
#define XPOWERS_CHIP_BQ25896

#include "esp_sleep.h"
#include <Wire.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"
#include "Adafruit_ADS1X15.h"   // or ADS1115 header
#include "EEPROM.h"
#include <Arduino.h>
#include <WiFi.h>
#include <XPowersLib.h>
#include "FreqCountESP.h"
#include "addresses.h"
#include "userInfo.h"
#include "equations.h"
#include "DFRobot_AS7341.h"
#include <Adafruit_NeoPixel.h>


#include "AXP.h"
#include <TimeLib.h>
#include "RTClib.h"
// #include <Arduino.h>
#include <SoftwareSerial.h>
#include "ESP32OTAPull.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// BLE START
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
// #include <Wire.h>
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "abebbebf-36e1-4688-b7f5-ea07361b26a8"
// BLE stop

//PMIC object
XPowersPPM PPM;

//PMIC Interrupt Request Flag
bool pmu_irq = false;

// Version Details
#define VERSION "0.0.1"
#define RELEASE_DATE "2024-12-15"

#ifndef CONFIG_PMU_SDA
#define CONFIG_PMU_SDA 21
#endif

#ifndef CONFIG_PMU_SCL
#define CONFIG_PMU_SCL 22
#endif

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif
const byte RX_PIN1 = 25, TX_PIN1 = 26;
int inputPin = 14;
int timerMs = 1000;
bool pmu_flag = 0;
//XPowersPMU PMU;

const uint8_t i2c_sda = CONFIG_PMU_SDA;
const uint8_t i2c_scl = CONFIG_PMU_SCL;
const uint8_t pmu_irq_pin = CONFIG_PMU_IRQ;

// spectrometer start
//  Define the pin that the LED strip is connected to
#define LED_PIN 13
//for 5 v
// #define LED_PIN 33

// Charging Update Variables
unsigned long chargeIconLastUpdate = 0;
unsigned long chargeIconInterval = 1000;

// Define the number of LEDs in the strip
#define NUM_LEDS 1

// Create a NeoPixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);

// initializing spectrometer sensor
DFRobot_AS7341 as7341;

//ADC and pH
DFRobot_ESP_PH_WITH_ADC ph;
Adafruit_ADS1115       ads;

const byte rxPin = 16;  // rx2
const byte txPin = 17;  // tx2
HardwareSerial dwin(1);

#define FILE_NAME "/c012data.csv"

unsigned long previousMillis = 0;
bool isFirstBattAssign = false;

// Variable to store the custom text
String customText = "";
float zeroValue = 0;
float zeroValue2 = 0;

int TH_Drops = 0;
int TA_Drops = 0;

int TH_B_C = 0;
int TA_C_D = 0;

int history_Index = 0;

struct RGB {
  int red;
  int green;
  int blue;
};
// spectrometer end

void setFlag(void) {
  pmu_flag = true;
}

AXP Axp;
RTC_PCF8563 rtc;

const int chipSelect = 5;
File file;
int fileSize = 0;

SoftwareSerial softSerial(RX_PIN1, TX_PIN1);
#define MAX_RECORDS 200
History historyRecords[MAX_RECORDS];
int currentRecordCount = 0;

// Function declarations
History parseData(String data);
void addHistoryRecord(String data);

String bleTest = "";
String bleTestId = "";
int bleDroplets = 0;
String bleReagent = "";
String bleZeroingTest = "";
bool underAppControl = false;

BLECharacteristic *pCharacteristic;
String BLEValue = "";
bool deviceConnected = false;
bool bleConductTest = false;
enum TestSyncStatus { TEST_LOCAL = 0,
                      TEST_SYNCED = 1 };
String latestTestAddressId = "";
String latestTestLat = "";
String latestTestLng = "";
class BLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    BLEValue = "";
    if (value.length() > 0) {
      Serial.println("*********");
      Serial.print("New value: ");
      for (int i = 0; i < value.length(); i++) {
        Serial.print(value[i]);
        BLEValue = BLEValue + value[i];
      }
      Serial.println();
      Serial.println("*********");
    }
  }
};

class BLESrvrCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    // pServer->startAdvertising(); // restart advertising after disconnecting
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  }
};

String deviceMacAddress = "";

void setup() {
  Serial.begin(115200);
  // *****************************************************************************************************************************************************
  // BLE start
  BLEDevice::init("Cluix C012");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new BLECallbacks());
  pCharacteristic->setValue("NULL");
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  // BLE stop
  // *****************************************************************************************************************************************************

  pinMode(12, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWrite(12, HIGH);
  FreqCountESP.begin(inputPin, timerMs);
  //axp_setup();
  bq_setup();
  //Axp.begin();
  softSerial.begin(9600);
  dwin.begin(115200, SERIAL_8N1, rxPin, txPin);

  //ph setup
  ph.begin();            
  ads.setGain(GAIN_ONE); // ±4.096 V
  ads.begin();

  // wifiSetup();
  delay(100);
  // Detect if I2C can communicate properly
  //Uncomment below for actual testing
  while (as7341.begin() != 0) {
    Serial.println("IIC init failed, please check if the wire connection is correct");
    delay(1000);
  }

  as7341.setAtime(29);
  as7341.setAstep(500);  //Original 599
  as7341.setAGAIN(7);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    // while (1)
    //   ;
  }

  if (rtc.lostPower()) {
    // Serial.println("RTC lost power, setting the time!");
    // Set the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    rtc.adjust(DateTime(2025, 4, 18, 18, 52, 0));
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("SD Card initialization failed!");
  } else {
    Serial.println("SD Card initialized.");
  }

  displayAllTheInformation();
  previousMillis = millis();
  // SD.remove(FILE_NAME);
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current time

  BLEAction();
  readScreen();

  if (underAppControl)
    dwin.write(ble_page_change, 10);

  if (currentMillis - previousMillis >= 10000 || !isFirstBattAssign) {
    if (!isFirstBattAssign) {
      isFirstBattAssign = true;
    }
    previousMillis = currentMillis;
    batteryPercent();
    PPM.getBusStatus();
    PPM.getFaultStatus();
       if (pmu_irq) {
        pmu_irq = false;

        // Get PPM interrupt status
        PPM.getFaultStatus();

        Serial.print("-> [");
        Serial.print(millis() / 1000);
        Serial.print("] ");

        if (PPM.isWatchdogFault()) {

            Serial.println("Watchdog Fault");

        } else if (PPM.isBoostFault()) {

            Serial.println("Boost Fault");

        } else if (PPM.isChargeFault()) {

            Serial.println("Charge Fault");

        } else if (PPM.isBatteryFault()) {

            Serial.println("Batter Fault");

        } else if (PPM.isNTCFault()) {

            Serial.print("NTC Fault:");
            Serial.print(PPM.getNTCStatusString());
            Serial.print(" Percentage:");
            Serial.print(PPM.getNTCPercentage()); Serial.println("%");
        } else {
            /*
            * When the battery is removed, INT will send an interrupt every 100ms. If the battery is not connected,
            * you can use PPM.disableCharge() to turn off the charging function.
            * */
            // PPM.disableCharge();

            Serial.println("Battery remove");
        }
    }
    // Serial.println(PPM.getChargeStatusString());
   
  }
  //Charging Icon
  if (millis() - chargeIconLastUpdate >= chargeIconInterval) {
    chargeIconLastUpdate = millis();
    if (PPM.getChargeStatusString() == "Fast Charging" || PPM.getChargeStatusString() == "Charge Termination Done" || PPM.getChargeStatusString() == "Pre-charge") {
      charging_icon[7] = 0x01;
    } else {
      charging_icon[7] = 0x00;
    }
    dwin.write(charging_icon, 8);
  }

}

float spectroReading(const char *whichWavelengthToReturn, int inputWavelength) {

  permissible_icon[7] = 0x00;
  dwin.write(permissible_icon, 8);
  RGB color = wavelengthToRGB(inputWavelength);

  strip.setPixelColor(0, color.red, color.green, color.blue);
  // strip.setBrightness(214.9450549);  
  strip.show();

  // Serial.println("rgb ");
  // Serial.println(color.red);

  // Serial.println(color.green);
  // Serial.println(color.blue);

  float outputWavelength = 0;

  int bufferSize = 10;
  float f1 = 0;
  float f2 = 0;
  float f3 = 0;
  float f4 = 0;
  float f5 = 0;
  float f6 = 0;
  float f7 = 0;
  float f8 = 0;
  float clear = 0;
  float NIR = 0;

  for (int x = 0; x < bufferSize; x++) {
    // Serial.println(x);
    DFRobot_AS7341::sModeOneData_t data1;
    DFRobot_AS7341::sModeTwoData_t data2;

    // Start spectrum measurement
    as7341.startMeasure(as7341.eF1F4ClearNIR);
    // Read the value of sensor data channel 0~5 under eF1F4ClearNIR mode
    data1 = as7341.readSpectralDataOne();

    as7341.startMeasure(as7341.eF5F8ClearNIR);
    data2 = as7341.readSpectralDataTwo();

// Can be removed in production code
    f1 += data1.ADF1;
    f2 += data1.ADF2;
    f3 += data1.ADF3;
    f4 += data1.ADF4;
    f5 += data2.ADF5;
    f6 += data2.ADF6;
    f7 += data2.ADF7;
    f8 += data2.ADF8;
    clear += data2.ADCLEAR;
    NIR += data2.ADNIR;
// till here

// Can remove redundant conditions since only two wavelength ranges are used 
//in equations.h for production code but use all for creating dataset and testing
    if (whichWavelengthToReturn == "f1") {
      outputWavelength += data1.ADF1;
    } else if (whichWavelengthToReturn == "f2") {
      outputWavelength += data1.ADF2;
    } else if (whichWavelengthToReturn == "f3") {
      outputWavelength += data1.ADF3;
    } else if (whichWavelengthToReturn == "f4") {
      outputWavelength += data1.ADF4;
    } else if (whichWavelengthToReturn == "f5") {
      outputWavelength += data2.ADF5;
    } else if (whichWavelengthToReturn == "f6") {
      outputWavelength += data2.ADF6;
    } else if (whichWavelengthToReturn == "f7") {
      outputWavelength += data2.ADF7;
    } else if (whichWavelengthToReturn == "f8") {
      outputWavelength += data2.ADF8;
    }
    delay(10);
  }

// For debugging and testing purpose only, can be removed in production code
  Serial.print("f1 ");
  Serial.println(f1 / 10);

  Serial.print("f2 ");
  Serial.println(f2 / 10);

  Serial.print("f3 ");
  Serial.println(f3 / 10);

  Serial.print("f4 ");
  Serial.println(f4 / 10);

  Serial.print("f5 ");
  Serial.println(f5 / 10);

  Serial.print("f6 ");
  Serial.println(f6 / 10);

  Serial.print("f7 ");
  Serial.println(f7 / 10);

  Serial.print("f8 ");
  Serial.println(f8 / 10);

  Serial.print("Clear ");
  Serial.println(clear / 10);

  Serial.print("NIR ");
  Serial.println(NIR / 10);

  strip.setPixelColor(0, 0, 0, 0);
  strip.show();

  return (outputWavelength / bufferSize);
}

void readScreen() {
  unsigned char Buffer[9];
  if (dwin.available()) {

    for (int i = 0; i <= 8; i++)  // this loop will store whole frame in buffer array.
    {
      Buffer[i] = dwin.read();
    }

    if (bleTest != "") {
      bleConductTest = true;
    } else {
      bleConductTest = false;
    }

    if (Buffer[4] == 0x01 && Buffer[5] == 0x10) {
      dwin.write(test_page_change, 10);
    } else if ((Buffer[4] == turbidity_test_add_first && Buffer[5] == turbidity_test_add_second) || bleTest == "Turbidity") {
      int intValue = turbidity_Test() * 100;
      //int intValue = flouride_Test() * 100;
      turbidity_result_reading[6] = highByte(intValue);
      turbidity_result_reading[7] = lowByte(intValue);
      dwin.write(turbidity_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 5.0);
      dwin.write(turbidity_result_reading, 8);
      processTestResult("Turbidity", String((float)intValue / 100.0), permissible, "1-5 ppm", "null", "NTU         .");
    } else if ((Buffer[4] == frc_test_add_first && Buffer[5] == frc_test_add_second) || bleTest == "FRC") {
      int intValue = frc_Test() * 100;
      frc_result_reading[6] = highByte(intValue);
      frc_result_reading[7] = lowByte(intValue);
      dwin.write(frc_result_reading, 8);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 1.0);
      dwin.write(frc_page_change, 10);
      processTestResult("FRC", String((float)intValue / 100.0), permissible, "0-1 ppm", "null", "ppm            .");

    } else if ((Buffer[4] == tds_test_add_first && Buffer[5] == tds_test_add_second) || bleTest == "TDS") {
      float intValue = tds_Test() * 0.54; 
      setTextDwin(0x5020, String(intValue) + "         ");
      dwin.write(tds_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 1000);
      processTestResult("TDS", String(intValue), permissible, "0-2k ppm", "null", "ppm        .");

    } else if ((Buffer[4] == iron_test_add_first && Buffer[5] == iron_test_add_second) || bleTest == "Iron") {
      int intValue = iron_Test() * 100;
      iron_result_reading[6] = highByte(intValue);
      iron_result_reading[7] = lowByte(intValue);
      dwin.write(iron_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 1.0);
      dwin.write(iron_result_reading, 8);
      processTestResult("Iron", String((float)intValue / 100.0), permissible, "0-1 ppm", "null", "ppm         .");


    } else if ((Buffer[4] == ph_test_add_first && Buffer[5] == ph_test_add_second) || bleTest == "pH") {
      int intValue = turbidity_Test() * 100;
      ph_result_reading[6] = highByte(intValue);
      ph_result_reading[7] = lowByte(intValue);
      dwin.write(ph_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, 6.5, 8.5);
      Serial.println(permissible);
      dwin.write(ph_result_reading, 8);
      processTestResult("pH", String((float)intValue / 100.0), permissible, "6.5-8.5 pH", "null", "pH        .");


    } else if ((Buffer[4] == th_test_add_first && Buffer[5] == th_test_add_second) || bleTest == "TH") {
      float intValue = th_Test();
      setTextDwin(th_result_add, String(intValue));
      String permissible = permissibleOrNot(intValue, -1.0, 600.0);
      processTestResult("TH", String(intValue), permissible, "0-600ppm", "null", "ppm        .");


    } else if ((Buffer[4] == phosphate_test_add_first && Buffer[5] == phosphate_test_add_second) || bleTest == "Lead") {
      int intValue = ((int)(phosphate_test() * 10) / 10.0) * 100;
      Serial.println(intValue);
      phosphate_result_reading[6] = highByte(intValue);
      phosphate_result_reading[7] = lowByte(intValue);
      dwin.write(phosphate_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 10, -0.01, 0.01);
      dwin.write(phosphate_result_reading, 8);
      processTestResult("Phosphate", String((float)intValue / 10.0), permissible, "0-0.01 ppm", "null", "ppm      .");


    } else if ((Buffer[4] == ta_test_add_first && Buffer[5] == ta_test_add_second) || bleTest == "TA") {
      float intValue = ta_Test();
      setTextDwin(ta_result_add, String(intValue));
      String permissible = permissibleOrNot(intValue, -1.0, 600.0);
      processTestResult("TA", String((float)intValue / 100.0), permissible, "0-600ppm", "null", "ppm       .");


    } else if ((Buffer[4] == nitrate_test_add_first && Buffer[5] == nitrate_test_add_second) || bleTest == "Nitrate") {
      int intValue = nitrate_Test() * 100;
      nitrate_result_reading[6] = highByte(intValue);
      nitrate_result_reading[7] = lowByte(intValue);
      dwin.write(nitrate_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 45.0);
      dwin.write(nitrate_result_reading, 8);
      processTestResult("Nitrate", String((float)intValue / 100.0), permissible, "0-45 ppm", "null", "ppm         .");


    } else if ((Buffer[4] == flouride_test_add_first && Buffer[5] == flouride_test_add_second) || bleTest == "Fluoride") {
      int intValue = flouride_Test() * 100;
      flouride_result_reading[6] = highByte(intValue);
      flouride_result_reading[7] = lowByte(intValue);
      dwin.write(flouride_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 1.5);
      dwin.write(flouride_result_reading, 8);
      processTestResult("Flouride", String((float)intValue / 100.0), permissible, "0-1.5 ppm", "null", "ppm       .");


    } else if ((Buffer[4] == copper_test_add_first && Buffer[5] == copper_test_add_second) || bleTest == "Copper") {
      int intValue = copper_Test() * 100;
      copper_result_reading[6] = highByte(intValue);
      copper_result_reading[7] = lowByte(intValue);
      dwin.write(copper_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 1.50);
      dwin.write(copper_result_reading, 8);
      processTestResult("Copper", String((float)intValue / 100.0), permissible, "0-1.5 ppm", "null", "ppm       .");

    } else if ((Buffer[4] == ec_test_add_first && Buffer[5] == ec_test_add_second) || bleTest == "EC") {
      float intValue = tds_Test();
      setTextDwin(0x6000, String(intValue) + "       ");
      dwin.write(ec_page_change, 10);
      String permissible = permissibleOrNot((float)intValue / 100, -1.0, 1000);
      processTestResult("EC ", String(intValue), permissible, "0-4k uS", "null", "uS        .");

    }


    // TH and TA add drops
    else if (Buffer[4] == TA_incremental_decremental_add_first && Buffer[5] == TA_incremental_decremental_add_second || bleDroplets != 0) {
      if (bleDroplets != 0) {
        TA_Drops = bleDroplets;
        bleDroplets = 0;
      } else {
        TA_Drops = Buffer[8];
      }
    } else if (Buffer[4] == TH_incremental_decremental_add_first && Buffer[5] == TH_incremental_decremental_add_second || bleDroplets != 0) {
      if (bleDroplets != 0) {
        TH_Drops = bleDroplets;
        bleDroplets = 0;
      } else {
        TH_Drops = Buffer[8];
      }
    }
    // TH and TA select B C D
    else if (Buffer[4] == 0x00 && Buffer[5] == TA_C_add || bleReagent == "TA_C") {
      TA_C_D = 0;
      bleReagent = "";
    } else if (Buffer[4] == 0x00 && Buffer[5] == TA_D_add || bleReagent == "TA_D") {
      TA_C_D = 1;
      bleReagent = "";
    } else if (Buffer[4] == 0x00 && Buffer[5] == TH_B_add || bleReagent == "TH_B") {
      TH_B_C = 0;
      bleReagent = "";
    } else if (Buffer[4] == 0x00 && Buffer[5] == TH_C_add || bleReagent == "TH_C") {
      TH_B_C = 1;
      bleReagent = "";
    }
    // zeroing
    else if (Buffer[4] == frc_zeroing_add_first && Buffer[5] == frc_zeroing_add_second || bleZeroingTest == "FRC") {
      Serial.println("zero frc");
      zeroValue = spectroReading(FRC_FILTER, FRC_WAVELENGTH);
      dwin.write(frc_zero_page_change, 10);
      Serial.println(zeroValue);
    } else if (Buffer[4] == iron_zeroing_add_first && Buffer[5] == iron_zeroing_add_second || bleZeroingTest == "Iron") {
      Serial.println("zero iron");
      zeroValue = spectroReading(IRON_FILTER, IRON_WAVELENGTH);
      dwin.write(iron_zero_page_change, 10);
      Serial.println(zeroValue);
    } else if (Buffer[4] == ph_zeroing_add_first && Buffer[5] == ph_zeroing_add_second || bleZeroingTest == "pH") {
      Serial.println("zero ph");
      zeroValue = spectroReading(PH_FILTER_520, PH_WAVELENGTH_520);
      dwin.write(ph_zero_page_change, 10);
      Serial.println(zeroValue);
    } else if (Buffer[4] == phosphate_zeroing_add_first && Buffer[5] == phosphate_zeroing_add_second || bleZeroingTest == "Phosphate") {
      Serial.println("zero phosphate");
      zeroValue = spectroReading(PHOSPHATE_FILTER, PHOSPHATE_WAVELENGTH);
      dwin.write(phosphate_zero_page_change, 10);
      Serial.println(zeroValue);
    } else if (Buffer[4] == nitrate_zeroing_add_first && Buffer[5] == nitrate_zeroing_add_second || bleZeroingTest == "Nitrate") {
      Serial.println("zero nitrate");
      zeroValue = spectroReading(NITRATE_FILTER, NITRATE_WAVELENGTH);
      dwin.write(nitrate_zero_page_change, 10);
      Serial.println(zeroValue);
    } else if (Buffer[4] == flouride_zeroing_add_first && Buffer[5] == flouride_zeroing_add_second || bleZeroingTest == "Fluoride") {
      Serial.println("zero flouride");
      zeroValue = spectroReading(FLUORIDE_FILTER, FLUORIDE_WAVELENGTH);
      dwin.write(flouride_zero_page_change, 10);
      Serial.println(zeroValue);
    } else if (Buffer[4] == copper_zeroing_add_first && Buffer[5] == copper_zeroing_add_second || bleZeroingTest == "Copper") {
      Serial.println("zero copper");
      zeroValue = spectroReading(COPPER_FILTER, COPPER_WAVELENGTH);
      dwin.write(copper_zero_page_change, 10);
      Serial.println(zeroValue);
    } else if (Buffer[4] == button_off_add_first && Buffer[5] == button_off_add_second) {
      delay(200);
      digitalWrite(12, LOW);
      Axp.PowerOff();
      Serial.println("power off");
    }

    //history
    else if (Buffer[4] == history_button_add_first && Buffer[5] == history_button_add_second) {
      readCSVFile();
      history_Index = 0;
      displayHistory();
    } else if (Buffer[4] == history_back_add_first && Buffer[5] == history_back_add_second) {
      if (history_Index < fileSize - 4) {
        history_Index = history_Index + 1;
      }
      displayHistory();
    } else if (Buffer[4] == history_next_add_first && Buffer[5] == history_next_add_second) {
      if (history_Index == 0) {
        dwin.write(home_page_change, 10);
      }
      // else if (Buffer[4] == wifi_connect_add_first && Buffer[5] == wifi_connect_add_second) {
      //     wifi_connect();
      // }
      displayHistory();
    } else if (Buffer[4] == history_box1_bt_first && Buffer[5] == history_box1_bt_second) {
      dwin.write(comman_test_page_change_history, 10);
      history_text(history_Index);
    } else if (Buffer[4] == history_box2_bt_first && Buffer[5] == history_box2_bt_second) {
      dwin.write(comman_test_page_change_history, 10);
      history_text(history_Index + 1);
    } else if (Buffer[4] == history_box3_bt_first && Buffer[5] == history_box3_bt_second) {
      dwin.write(comman_test_page_change_history, 10);
      history_text(history_Index + 2);
    } else if (Buffer[4] == history_box4_bt_first && Buffer[5] == history_box4_bt_second) {
      dwin.write(comman_test_page_change_history, 10);
      history_text(history_Index + 3);
    } else if (Buffer[4] == 0x00 && Buffer[5] == update_button_second) {
      Serial.println("update");
    } 
  }

  if (bleZeroingTest != "") {
    bleZeroingTest = "";
    String msg = "ZERO_CALIBRATION_RESULT,1,";
    sendBleMessage(msg);
  }
  // resetting values
  bleTest = "";
  bleTestId = "";
  bleZeroingTest = "";
  latestTestAddressId = "";
  latestTestLat = "";
  latestTestLng = "";

  delay(10);
}

float turbidity_Test() {
  float turb = 0;
  digitalWrite(27, HIGH);
  delay(2000);

  if (FreqCountESP.available()) {
    uint32_t tempfrequency = 0;
    for (int i = 0; i < 10; i++) {
      tempfrequency += FreqCountESP.read();
      Serial.println(FreqCountESP.read());
      delay(100);
    }
    Serial.print("Turbidity : ");
    Serial.println(tempfrequency / 10);
    turb = tempfrequency / 10;
    float turbSlope = 0.0;
    float turbIntercept = 0.0;
    for (int i = 0; i < numberOfTurb; i++) {
      if (deviceMacAddress.equals(turbidityList[i].mac_Address)) {
        turbSlope = turbidityList[i].slope;
        turbIntercept = turbidityList[i].intercept;
      }
    }
    turb = turbSlope * turb + turbIntercept;
    Serial.println(turb);

    delayMicroseconds(1000);
  }
  digitalWrite(27, LOW);
  if (turb < 0) {
    return 0;
  } else if (turb > 10) {
    return 10;
  }
  return turb;
}
// - - - - - - - - - - - - - - - - - - - -TDS - - - - - - - - - - - - -
float tds_Test() {
  int value = 0;
  int sampleCount = 0;
  softSerial.write('1');
  for (int i = 0; i < 10; i++) {
    String dataS = softSerial.readString();
    int startIndex = dataS.indexOf(' ') + 1;
    int endIndex = dataS.indexOf(' ', startIndex);
    String numberStr = dataS.substring(startIndex, endIndex);
    int number = numberStr.toInt();
    if (number != 0 && i != 0) {
      value += number;
      sampleCount++;
    }
    Serial.println(number);
    delay(100);
  }
  softSerial.write('0');

  if (value != 0) {
    value = value / sampleCount;
  } else {
    return 0;
  }
  float tdsFactor = 0.0;
  if (value < 4000) {
    for (int i = 0; i < numberOfTurb; i++) {
      if (deviceMacAddress.equals(tdsList[i].mac_Address)) {
        tdsFactor = tdsList[i].factor1;
        break;
      } else {
        tdsFactor = 0.0721;  
      }
    }
  } else if (value > 17000) {
    for (int i = 0; i < numberOfTurb; i++) {
      if (deviceMacAddress.equals(tdsList[i].mac_Address)) {
        tdsFactor = tdsList[i].factor3;
        break;
      } else {
        tdsFactor = 0.087;
      }
    }
  } else {
    for (int i = 0; i < numberOfTurb; i++) {
      if (deviceMacAddress.equals(tdsList[i].mac_Address)) {
        tdsFactor = tdsList[i].factor2;
        break;
      } else {
        tdsFactor = 0.087;
      }
    }
  }
  // float conductivity = value * TDS_CONDUCTIVITY_FACTOR;
  float conductivity = value * tdsFactor;
  return conductivity;
}

float th_Test() {
  float value = 0;

  if (TH_B_C == 0) {
    value = TH_Drops * TH_LOW_CONCENTRATION;
  } else {
    value = TH_Drops * TH_HIGH_CONCENTRATION;
  }
  Serial.println(value);
  return value;
}

float ta_Test() {
  float value = 0;

  if (TA_C_D == 0) {
    value = TA_Drops * TA_LOW_CONCENTRATION;
  } else {
    value = TA_Drops * TA_HIGH_CONCENTRATION;
  }
  Serial.println(value);

  return value;
}

float frc_Test() {
  float i_initial = log10(zeroValue);
  Serial.println("Starting : ");

  float i_final = spectroReading(FRC_FILTER, FRC_WAVELENGTH);

  Serial.println(i_final);
  i_final = log10(i_final);
  Serial.println(i_final);

  float value = FRC_SLOPE * (i_initial - i_final) + FRC_INTERCEPT;
  Serial.println(value);

  if (value < 0) {
    value = 0;
  }
  return value;
}

float iron_Test() {
  float i_initial = log10(zeroValue);
  Serial.println(zeroValue);
  Serial.println("Starting : ");
  float i_final = spectroReading(IRON_FILTER, IRON_WAVELENGTH);

  Serial.println(i_final);
  i_final = log10(i_final);
  Serial.println(i_final);

  float value = IRON_SLOPE * (i_initial - i_final) + IRON_INTERCEPT;
  Serial.println(value);

  if (value < 0) {
    value = 0;
  }
  return value;
}

// float ph_Test() {
//   float i_initial_450 = log10(zeroValue);
//   Serial.println("Starting 450 : ");
//   float i_final_450 = spectroReading(PH_FILTER_450, PH_WAVELENGTH_450);

//   Serial.println(i_final_450);
//   i_final_450 = log10(i_final_450);

//   float i_initial_520 = log10(zeroValue);
//   Serial.println("Starting 520 : ");
//   float i_final_520 = spectroReading(PH_FILTER_520, PH_WAVELENGTH_520);

//   Serial.println(i_final_520);
//   i_final_520 = log10(i_final_520);

//   float logRatio = (i_initial_520 - i_final_520) / (i_initial_450 - i_final_450);

//   float value = PH_SLOPE * logRatio + PH_INTERCEPT;

//   if (value < 0) {
//     value = 0;
//   }
//   return value;
// }

float ph_Test() {
  float PhValue=0.0;
  static unsigned long t0 = millis();
  for(int i=0;i<15;i++){
  if (millis() - t0 > 1000U) {
    t0 = millis();

    // 1) read pH sensor on channel 1
    int16_t rawPh = ads.readADC_SingleEnded(1)/10;
    float vPh = rawPh;  // volts
    Serial.print("pH voltage: ");
    Serial.print(vPh, 4);
    Serial.print(" V  |  ");

    // 2) read temperature from NTC on channel 2
    float tempC = 25.0;
    Serial.print("Temp: ");
    Serial.print(tempC, 2);
    Serial.print(" °C  |  ");

    // 3) calculate pH with temperature compensation
    float phValue = ph.readPH(vPh, tempC);
    Serial.print("pH: ");
    Serial.println(phValue, 4);
    // float PhValue = 0.0157* (vPh )- 16.4581; 
    PhValue = 0.0109* (vPh )-11.4956; 
    Serial.print("pH2: ");
    Serial.println(PhValue, 4);
    Serial.println(vPh, 4);
  }
  delay(1000);
}
return PhValue;
}

float phosphate_test() {
  float i_initial = log10(zeroValue);
  Serial.println("Starting : ");
  float i_final = spectroReading(PHOSPHATE_FILTER, PHOSPHATE_WAVELENGTH);

  Serial.println(i_final);
  i_final = log10(i_final);
  Serial.println(i_final);

  float value = PHOSPHATE_SLOPE * (i_initial - i_final) + PHOSPHATE_INTERCEPT;
  Serial.println(value);

  if (value < 0) {
    value = 0;
  }
  return value;
}

float nitrate_Test() {
  float i_initial = log10(zeroValue);
  Serial.println("Starting : ");
  float i_final = spectroReading(NITRATE_FILTER, NITRATE_WAVELENGTH);

  Serial.println(i_final);
  i_final = log10(i_final);
  Serial.println(i_final);

  float value = NITRATE_SLOPE * (i_initial - i_final) + NITRATE_INTERCEPT;
  Serial.println(value);

  if (value < 0) {
    value = 0;
  }
  return value;
}

float flouride_Test() {
  float i_initial = log10(zeroValue);
  Serial.println("Starting : ");
  float i_final = spectroReading(FLUORIDE_FILTER, FLUORIDE_WAVELENGTH);

  i_final = log10(i_final);

  float value = (i_initial - i_final);

  value = 1 / value;

  value = FLUORIDE_SLOPE * value + FLUORIDE_INTERCEPT;
  Serial.println(value);

  if (value < 0) {
    value = 0;
  }
  return value;
}

float copper_Test() {
  float i_initial = log10(zeroValue);
  Serial.println("Starting : ");
  float i_final = spectroReading(COPPER_FILTER, COPPER_WAVELENGTH);

  Serial.println(i_final);
  i_final = log10(i_final);
  Serial.println(i_final);

  float value = COPPER_SLOPE * (i_initial - i_final) + COPPER_INTERCEPT;
  Serial.println(value);

  if (value < 0) {
    value = 0;
  }
  return value;
}

RGB wavelengthToRGB(double wavelength) {
  double Gamma = 0.80;
  double IntensityMax = 255.0;
  double factor;
  double red, green, blue;

  if ((wavelength >= 380) && (wavelength < 440)) {
    red = -(wavelength - 440) / (440 - 380);
    green = 0.0;
    blue = 1.0;
  } else if ((wavelength >= 440) && (wavelength < 490)) {
    red = 0.0;
    green = (wavelength - 440) / (490 - 440);
    blue = 1.0;
  } else if ((wavelength >= 490) && (wavelength < 510)) {
    red = 0.0;
    green = 1.0;
    blue = -(wavelength - 510) / (510 - 490);
  } else if ((wavelength >= 510) && (wavelength < 580)) {
    red = (wavelength - 510) / (580 - 510);
    green = 1.0;
    blue = 0.0;
  } else if ((wavelength >= 580) && (wavelength < 645)) {
    red = 1.0;
    green = -(wavelength - 645) / (645 - 580);
    blue = 0.0;
  } else if ((wavelength >= 645) && (wavelength < 781)) {
    red = 1.0;
    green = 0.0;
    blue = 0.0;
  } else {
    red = 0.0;
    green = 0.0;
    blue = 0.0;
  }

  // Let the intensity fall off near the vision limits
  if ((wavelength >= 380) && (wavelength < 420)) {
    factor = 0.3 + 0.7 * (wavelength - 380) / (420 - 380);
  } else if ((wavelength >= 420) && (wavelength < 701)) {
    factor = 1.0;
  } else if ((wavelength >= 701) && (wavelength < 781)) {
    factor = 0.3 + 0.7 * (780 - wavelength) / (780 - 700);
  } else {
    factor = 0.0;
  }

  int r = (red == 0.0) ? 0 : round(IntensityMax * pow(red * factor, Gamma));
  int g = (green == 0.0) ? 0 : round(IntensityMax * pow(green * factor, Gamma));
  int b = (blue == 0.0) ? 0 : round(IntensityMax * pow(blue * factor, Gamma));

  RGB color = { r, g, b };
  return color;
}

// void axp_setup() {

//   bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, i2c_sda, i2c_scl);

//   PMU.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_4V36);

//   PMU.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_1500MA);

//   uint16_t vol = PMU.getSysPowerDownVoltage();
//   // Serial.printf("->  getSysPowerDownVoltage:%u\n", vol);

//   PMU.setSysPowerDownVoltage(2600);

//   vol = PMU.getSysPowerDownVoltage();
//   // Serial.printf("->  getSysPowerDownVoltage:%u\n", vol);

//   PMU.setDC1Voltage(3300);
//   // Serial.printf("DC1  : %s   Voltage:%u mV \n", PMU.isEnableDC1() ? "+" : "-", PMU.getDC1Voltage());

//   PMU.setDC2Voltage(3300);
//   // Serial.printf("DC2  : %s   Voltage:%u mV \n", PMU.isEnableDC2() ? "+" : "-", PMU.getDC2Voltage());

//   PMU.setDC3Voltage(3300);
//   // Serial.printf("DC3  : %s   Voltage:%u mV \n", PMU.isEnableDC3() ? "+" : "-", PMU.getDC3Voltage());

//   PMU.setDC4Voltage(3300);
//   // Serial.printf("DC4  : %s   Voltage:%u mV \n", PMU.isEnableDC4() ? "+" : "-", PMU.getDC4Voltage());

//   PMU.setDC5Voltage(3300);
//   // Serial.printf("DC5  : %s   Voltage:%u mV \n", PMU.isEnableDC5() ? "+" : "-", PMU.getDC5Voltage());

//   PMU.setALDO1Voltage(1500);
//   // Serial.printf("ALDO1  : %s   Voltage:%u mV \n", PMU.isEnableALDO1() ? "+" : "-", PMU.getALDO1Voltage());

//   PMU.setALDO2Voltage(1800);
//   // Serial.printf("ALDO2  : %s   Voltage:%u mV \n", PMU.isEnableALDO2() ? "+" : "-", PMU.getALDO2Voltage());

//   PMU.setALDO3Voltage(1800);
//   // Serial.printf("ALDO3  : %s   Voltage:%u mV \n", PMU.isEnableALDO3() ? "+" : "-", PMU.getALDO3Voltage());

//   PMU.setALDO4Voltage(3300);
//   // Serial.printf("ALDO4  : %s   Voltage:%u mV \n", PMU.isEnableALDO4() ? "+" : "-", PMU.getALDO4Voltage());

//   PMU.setBLDO1Voltage(3300);
//   // Serial.printf("BLDO1  : %s   Voltage:%u mV \n", PMU.isEnableBLDO1() ? "+" : "-", PMU.getBLDO1Voltage());

//   PMU.setBLDO2Voltage(3300);
//   // Serial.printf("BLDO2  : %s   Voltage:%u mV \n", PMU.isEnableBLDO2() ? "+" : "-", PMU.getBLDO2Voltage());

//   PMU.setCPUSLDOVoltage(1000);

//   PMU.setDLDO1Voltage(3300);
//   // Serial.printf("DLDO1  : %s   Voltage:%u mV \n", PMU.isEnableDLDO1() ? "+" : "-", PMU.getDLDO1Voltage());

//   PMU.setDLDO2Voltage(3300);

//   PMU.enableDC1();
//   PMU.enableDC2();
//   PMU.enableDC3();
//   //PMU.disableDC3;
//   PMU.disableDC4();
//   // PMU.enableDC5();
//   PMU.disableDC5();

//   PMU.enableALDO1();
//   //PMU.disableALDO1();
//   PMU.enableALDO2();
//   PMU.enableALDO3();
//   // PMU.enableALDO4();
//   PMU.disableALDO4();
//   PMU.enableBLDO1();
//   PMU.enableBLDO2();
//   // PMU.disableBLDO2();

//   // PMU.enableCPUSLDO();
//   PMU.disableCPUSLDO();
//   PMU.enableDLDO1();
//   PMU.enableDLDO2();

//   // Set the time of pressing the button to turn off
//   PMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
//   uint8_t opt = PMU.getPowerKeyPressOffTime();
//   // Serial.print("PowerKeyPressOffTime:");
//   switch (opt) {
//     case XPOWERS_POWEROFF_4S:
//       // Serial.println("4 Second");
//       break;
//     case XPOWERS_POWEROFF_6S:
//       // Serial.println("6 Second");
//       break;
//     case XPOWERS_POWEROFF_8S:
//       // Serial.println("8 Second");
//       break;
//     case XPOWERS_POWEROFF_10S:
//       // Serial.println("10 Second");
//       break;
//     default:
//       break;
//   }
//   // Set the button power-on press time
//   PMU.setPowerKeyPressOnTime(XPOWERS_POWERON_128MS);
//   opt = PMU.getPowerKeyPressOnTime();
//   // Serial.print("PowerKeyPressOnTime:");
//   switch (opt) {
//     case XPOWERS_POWERON_128MS:
//       // Serial.println("128 Ms");
//       break;
//     case XPOWERS_POWERON_512MS:
//       // Serial.println("512 Ms");
//       break;
//     case XPOWERS_POWERON_1S:
//       // Serial.println("1 Second");
//       break;
//     case XPOWERS_POWERON_2S:
//       // Serial.println("2 Second");
//       break;
//     default:
//       break;
//   }

//   // Serial.println("===========================================================================");
//   //
//   bool en;

//   // DCDC 120%(130%) high voltage turn off PMIC function
//   en = PMU.getDCHighVoltagePowerDownEn();
//   // Serial.print("getDCHighVoltagePowerDownEn:");
//   // Serial.println(en ? "ENABLE" : "DISABLE");
//   // DCDC1 85% low voltage turn off PMIC function
//   en = PMU.getDC1LowVoltagePowerDownEn();
//   // Serial.print("getDC1LowVoltagePowerDownEn:");
//   // Serial.println(en ? "ENABLE" : "DISABLE");
//   // DCDC2 85% low voltage turn off PMIC function
//   en = PMU.getDC2LowVoltagePowerDownEn();
//   // Serial.print("getDC2LowVoltagePowerDownEn:");
//   // Serial.println(en ? "ENABLE" : "DISABLE");
//   // DCDC3 85% low voltage turn off PMIC function
//   en = PMU.getDC3LowVoltagePowerDownEn();
//   // Serial.print("getDC3LowVoltagePowerDownEn:");
//   // Serial.println(en ? "ENABLE" : "DISABLE");
//   // DCDC4 85% low voltage turn off PMIC function
//   en = PMU.getDC4LowVoltagePowerDownEn();
//   // Serial.print("getDC4LowVoltagePowerDownEn:");
//   // Serial.println(en ? "ENABLE" : "DISABLE");
//   // DCDC5 85% low voltage turn off PMIC function
//   en = PMU.getDC5LowVoltagePowerDownEn();
//   // Serial.print("getDC5LowVoltagePowerDownEn:");
//   // Serial.println(en ? "ENABLE" : "DISABLE");

//   // PMU.setDCHighVoltagePowerDown(true);
//   // PMU.setDC1LowVoltagePowerDown(true);
//   // PMU.setDC2LowVoltagePowerDown(true);
//   // PMU.setDC3LowVoltagePowerDown(true);
//   // PMU.setDC4LowVoltagePowerDown(true);
//   // PMU.setDC5LowVoltagePowerDown(true);

//   // It is necessary to disable the detection function of the TS pin on the board
//   // without the battery temperature detection function, otherwise it will cause abnormal charging
//   PMU.disableTSPinMeasure();

//   // PMU.enableTemperatureMeasure();

//   // Enable internal ADC detection
//   PMU.enableBattDetection();
//   PMU.enableVbusVoltageMeasure();
//   PMU.enableBattVoltageMeasure();
//   PMU.enableSystemVoltageMeasure();

//   /*
//       The default setting is CHGLED is automatically controlled by the PMU.
//     - XPOWERS_CHG_LED_OFF,
//     - XPOWERS_CHG_LED_BLINK_1HZ,
//     - XPOWERS_CHG_LED_BLINK_4HZ,
//     - XPOWERS_CHG_LED_ON,
//     - XPOWERS_CHG_LED_CTRL_CHG,
//     * */
//   PMU.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

//   // Force add pull-up
//   pinMode(pmu_irq_pin, INPUT_PULLUP);
//   attachInterrupt(pmu_irq_pin, setFlag, FALLING);

//   // Disable all interrupts
//   PMU.disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
//   // Clear all interrupt flags
//   PMU.clearIrqStatus();
//   // Enable the required interrupt function
//   PMU.enableIRQ(
//     XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |        // BATTERY
//     XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |      // VBUS
//     XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |         // POWER KEY
//     XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ |   // CHARGE
//     XPOWERS_AXP2101_WARNING_LEVEL1_IRQ | XPOWERS_AXP2101_WARNING_LEVEL2_IRQ  // Low battery warning
//                                                                              //  XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
//   );

//   // Set the precharge charging current
//   PMU.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_200MA);

//   // Set stop charging termination current
//   PMU.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);

//   // Set constant current charge current limit
//   if (!PMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_1000MA)) {
//     Serial.println("Setting Charger Constant Current Failed!");
//   }

//   const uint16_t currTable[] = {
//     0, 0, 0, 0, 100, 125, 150, 175, 200, 300, 400, 500, 600, 700, 800, 900, 1000
//   };
//   uint8_t val = PMU.getChargerConstantCurr();
//   // Serial.print("Setting Charge Target Current : ");
//   // Serial.println(currTable[val]);

//   // Set charge cut-off voltage
//   PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

//   const uint16_t tableVoltage[] = {
//     0, 4000, 4100, 4200, 4350, 4400, 255
//   };
//   val = PMU.getChargeTargetVoltage();
//   // Serial.print("Setting Charge Target Voltage : ");
//   // Serial.println(tableVoltage[val]);

//   // Set the power level to be lower than 15% and send an interrupt to the host`
//   PMU.setLowBatWarnThreshold(10);

//   // Set the power level to be lower than 5% and turn off the PMU supply
//   PMU.setLowBatShutdownThreshold(5);

//   /*
//     Turn on the learning battery curve,
//     And write the learned battery curve into the ROM
//     */
//   PMU.fuelGaugeControl(true, true);

//   Serial.println();

//   // Set charge cut-off voltage
//   PMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);

//   // Set the watchdog trigger event type
//   PMU.setWatchdogConfig(XPOWERS_AXP2101_WDT_IRQ_TO_PIN);
//   // Set watchdog timeout
//   PMU.setWatchdogTimeout(XPOWERS_AXP2101_WDT_TIMEOUT_4S);
//   // Enable watchdog to trigger interrupt event
//   PMU.enableWatchdog();

//   // PMU.disableWatchdog();

//   // Enable Button Battery charge
//   PMU.enableButtonBatteryCharge();

//   // Set Button Battery charge voltage
//   PMU.setButtonBatteryChargeVoltage(3300);
// }

void IRAM_ATTR pmuISR() {
  pmu_irq = true;
}

void bq_setup(){
  // Initialize XPowersLib for BQ25896
  Wire.begin(i2c_sda, i2c_scl);
  delay(10);
  
  if (!PPM.init(Wire, i2c_sda, i2c_scl, BQ25896_SLAVE_ADDRESS)) {
    Serial.println("PPM is not online. Halting...");
    while (1) { delay(1000); }
  }
  Serial.println("XPowersLib (BQ25896) initialized successfully.");

  //PPM.disableAutomaticInputDetection();
  // Configure PMIC parameters
  PPM.setSysPowerDownVoltage(3300);
  PPM.setInputCurrentLimit(3250);
  Serial.printf("Input current limit: %d mA\n", PPM.getInputCurrentLimit());
  PPM.disableCurrentLimitPin();
  PPM.setChargeTargetVoltage(4208); // Target ~4.208V (adjust if needed)
  PPM.setPrechargeCurr(64);         // Precharge current: 64 mA
  PPM.setChargerConstantCurr(3000); // Fast-charge current: 3000 mA
  Serial.printf("Fast-charge current: %d mA\n", PPM.getChargerConstantCurr());
  PPM.enableMeasure();
  PPM.enableCharge();
  PPM.disableAutomaticInputDetection();//try adapter charge
  // Attach PMU IRQ
  pinMode(pmu_irq_pin, INPUT_PULLUP);
  attachInterrupt(pmu_irq_pin, pmuISR, FALLING);
  // bring up I2C
  // 0) bring up I²C
  // 1) init I2C + PMIC
  // Wire.begin(i2c_sda, i2c_scl);
  // delay(10);
  // if (!PPM.init(Wire, i2c_sda, i2c_scl, BQ25896_SLAVE_ADDRESS)) {
  //   Serial.println("PPM init failed!");
  //   while (1) delay(1000);
  // }

  // // 2) enable auto‑measure + auto‑charge so VBUS_STAT is valid
  // PPM.enableMeasure();
  // PPM.enableMeasure();
  


  // PPM.enableCharge();
  // delay(200);

  // // 3) detect USB port type
  // String port = PPM.getBusStatusString();  // "SDP", "CDP", or "DCP"
  // Serial.printf("VBUS port = %s\n", port.c_str());

  // // 4) pick a “desired” limit
  // uint16_t desired;
  // if (port == "USB Host SDP")      desired = 500;
  // else if (port == "Adapter") desired = 1500;
  // else                    desired = 500;  // DCP or unknown

  // // 5) round to 64 mA step
  // auto make64 = [](uint16_t x){
  //   return (uint16_t)((x + 32) / 64) * 64;
  // };
  // uint16_t step64 = make64(desired);

  // // 6) program IINLIM & charger current
  // PPM.setInputCurrentLimit(step64);
  // PPM.disableCurrentLimitPin();
  // PPM.setChargeTargetVoltage(4208);
  // PPM.setPrechargeCurr(64);
  // PPM.setChargerConstantCurr(step64);
  // Serial.printf("IINLIM = %u mA, FastCharge = %u mA\n",
  //               PPM.getInputCurrentLimit(),
  //               PPM.getChargerConstantCurr());

  // // 7) finally re‑enable charging
  // PPM.enableMeasure();
  // PPM.enableCharge();

  // // (optional) IRQ on VBUS changes
  // pinMode(pmu_irq_pin, INPUT_PULLUP);
  // attachInterrupt(pmu_irq_pin, pmuISR, FALLING);
}

float calculateBatteryPercent() {
  float batteryVoltage = PPM.getBattVoltage() / 1000.0;
  // Serial.println(batteryVoltage);
  float batteryPercentage = (batteryVoltage - 3.0) * (100.0 / (4.1 - 3.0));
  if (batteryVoltage < 3.0) {
    return 0;
  } else if (batteryVoltage > 4.1) {
    return 100;
  }
  return batteryPercentage;
}

void batteryPercent() {
  int batteryPercentage = (int)calculateBatteryPercent();

  setTextDwin(0x8070, String(batteryPercentage) + "%              ");
  // Serial.println(batteryPercentage);
  if (batteryPercentage <= 0) {
    // Axp.PowerOff();
    dwin.write(battery_icon_0, 8);
  } else if (batteryPercentage <= 20 && batteryPercentage > 0) {
    dwin.write(battery_icon_20, 8);
  } else if (batteryPercentage <= 40 && batteryPercentage > 20) {
    dwin.write(battery_icon_40, 8);
  } else if (batteryPercentage <= 60 && batteryPercentage > 40) {
    dwin.write(battery_icon_60, 8);
  } else if (batteryPercentage <= 80 && batteryPercentage > 60) {
    dwin.write(battery_icon_80, 8);
  } else if (batteryPercentage <= 100 && batteryPercentage > 80) {
    dwin.write(battery_icon_100, 8);
  }
  
  DateTime now = rtc.now();

  setTextDwin(time_address, String(now.hour()) + ":" + (now.minute() < 10 ? "0" + String(now.minute()) : String(now.minute())));
  //setTextDwin(time_address,"19:33");
  // permissible_icon[7] = 0x00;
  // dwin.write(permissible_icon, 8);
  // dwin.write(permissible_icon, 8);
}

void setTextDwin(uint16_t vpAddress, String text) {
  uint8_t buffer[100];  // Buffer to hold the command
  int len = text.length();

  // Construct the command frame
  buffer[0] = 0x5A;                     // Start byte 1
  buffer[1] = 0xA5;                     // Start byte 2
  buffer[2] = len + 3;                  // Data length (text length + 3 for the address and control bytes)
  buffer[3] = 0x82;                     // Command for writing to a VP address
  buffer[4] = (vpAddress >> 8) & 0xFF;  // VP address high byte
  buffer[5] = vpAddress & 0xFF;         // VP address low byte

  // Copy the text data into the buffer
  for (int i = 0; i < len; i++) {
    buffer[6 + i] = text[i];
  }

  // Send the buffer to the DWIN screen
  dwin.write(buffer, len + 6);
}

void displayAllTheInformation() {
  deviceMacAddress = WiFi.macAddress();
  Serial.println(deviceMacAddress);
  for (int i = 0; i < 20; i++) {
    if (deviceMacAddress.equals(deviceInformationList[i].deviceMacAddress)) {
      setTextDwin(device_serial_text_vp_add, deviceInformationList[i].deviceSerialNumber);
      setTextDwin(build_number_text_vp_add, deviceInformationList[i].buildNumber);
      setTextDwin(mac_address_text_vp_add, deviceInformationList[i].deviceMacAddress);
      setTextDwin(profile_name_text_vp_add, deviceInformationList[i].userName);
      setTextDwin(phone_number_text_vp_add, deviceInformationList[i].phoneNumber);
      setTextDwin(location_text_vp_add, deviceInformationList[i].location);
      setTextDwin(registration_date_text_vp_add, deviceInformationList[i].registrationDate);
      setTextDwin(home_page_username_add, deviceInformationList[i].userName);
      setTextDwin(home_page_username_add, deviceInformationList[i].userName);
    }
  }
  setTextDwin(version_text_vp_add, "0.0.1");
}

void displayHistory() {
  String sno1 = "";
  String sno2 = "";
  String sno3 = "";
  String sno4 = "";

  if (history_Index + 1 > 0 && history_Index + 1 < 10) {
    sno1 = "  " + String(history_Index + 1);
  } else if (history_Index + 1 >= 10 && history_Index + 1 < 100) {
    sno1 = " " + String(history_Index + 1);
  } else if (history_Index + 1 >= 100 && history_Index + 1 < 1000) {
    sno1 = "" + String(history_Index + 1);
  }
  setTextDwin(history_box1_sno, sno1);
  setTextDwin(history_box1_p, historyRecords[history_Index].dateTime);
  setTextDwin(history_box1_dt, historyRecords[history_Index].permissible);
  setTextDwin(history_box1_result, historyRecords[history_Index].testName + " " + historyRecords[history_Index].value + " " + historyRecords[history_Index].unit);

  if (history_Index + 2 > 0 && history_Index + 2 < 10) {
    sno2 = "  " + String(history_Index + 2);
  } else if (history_Index + 2 >= 10 && history_Index + 2 < 100) {
    sno2 = " " + String(history_Index + 2);
  } else if (history_Index + 2 >= 100 && history_Index + 2 < 1000) {
    sno2 = "" + String(history_Index + 2);
  }
  setTextDwin(history_box2_sno, sno2);
  setTextDwin(history_box2_dt, historyRecords[history_Index + 1].dateTime);
  setTextDwin(history_box2_p, historyRecords[history_Index + 1].permissible);
  setTextDwin(history_box2_result, historyRecords[history_Index + 1].testName + " " + historyRecords[history_Index + 1].value + " " + historyRecords[history_Index + 1].unit);

  if (history_Index + 3 > 0 && history_Index + 3 < 10) {
    sno3 = "  " + String(history_Index + 3);
  } else if (history_Index + 3 >= 10 && history_Index + 3 < 100) {
    sno3 = " " + String(history_Index + 3);
  } else if (history_Index + 3 >= 100 && history_Index + 3 < 1000) {
    sno3 = "" + String(history_Index + 3);
  }

  setTextDwin(history_box3_sno, sno3);
  setTextDwin(history_box3_dt, historyRecords[history_Index + 2].dateTime);
  setTextDwin(history_box3_p, historyRecords[history_Index + 2].permissible);
  setTextDwin(history_box3_result, historyRecords[history_Index + 2].testName + " " + historyRecords[history_Index + 2].value + " " + historyRecords[history_Index + 2].unit);

  if (history_Index + 4 > 0 && history_Index + 4 < 10) {
    sno4 = "  " + String(history_Index + 4);
  } else if (history_Index + 4 >= 10 && history_Index + 4 < 100) {
    sno4 = " " + String(history_Index + 4);
  } else if (history_Index + 4 >= 100 && history_Index + 4 < 1000) {
    sno4 = "" + String(history_Index + 4);
  }

  setTextDwin(history_box4_sno, sno4);
  setTextDwin(history_box4_dt, historyRecords[history_Index + 3].dateTime);
  setTextDwin(history_box4_p, historyRecords[history_Index + 3].permissible);
  setTextDwin(history_box4_result, historyRecords[history_Index + 3].testName + " " + historyRecords[history_Index + 3].value + " " + historyRecords[history_Index + 3].unit);
}

String permissibleOrNot(float value, float lowerRange, float upperRange) {

  if (value > lowerRange && value < upperRange) {
    permissible_icon[7] = 0x00;
    dwin.write(permissible_icon, 8);
    return "yes  ";
  } else if (value <= lowerRange) {
    permissible_icon[7] = 0x01;
    dwin.write(permissible_icon, 8);
    return "no  ";
  } else if (value >= upperRange) {
    permissible_icon[7] = 0x01;
    dwin.write(permissible_icon, 8);
    return "no  ";
  } else {
    permissible_icon[7] = 0x00;
    dwin.write(permissible_icon, 8);
    return "null";
  }
}

void history_text(int index) {
  setTextDwin(history_screen_test_name, String(historyRecords[index].serial + 1) + ": " + historyRecords[index].testName);
  setTextDwin(history_screen_test_dt, historyRecords[index].dateTime);
  setTextDwin(history_screen_test_result, historyRecords[index].value + " " + historyRecords[index].unit);
  setTextDwin(history_screen_test_permissible_range, historyRecords[index].range);
  if (historyRecords[index].permissible.equals("yes  ")) {
    permissible_icon[7] = 0x00;
    dwin.write(permissible_icon, 8);
  } else {
    permissible_icon[7] = 0x01;
    dwin.write(permissible_icon, 8);
  }
}

void UPDATEDEVICE() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  delay(2000);
  if (!WiFi.isConnected()) {
    ESP32OTAPull ota;
    int ret = ota.CheckForOTAUpdate("http://15.206.94.169:8000/C011.json", VERSION);
    String error = errtext(ret);
  }
}

String errtext(int code) {
  switch (code) {
    case ESP32OTAPull::UPDATE_AVAILABLE:
      return "An update is available but wasn't installed";
    case ESP32OTAPull::NO_UPDATE_PROFILE_FOUND:
      return "No profile matches";
    case ESP32OTAPull::NO_UPDATE_AVAILABLE:
      return "Profile matched, but update not applicable";
    case ESP32OTAPull::UPDATE_OK:
      return "An update was done, but no reboot";
    case ESP32OTAPull::HTTP_FAILED:
      return "HTTP GET failure";
    case ESP32OTAPull::WRITE_ERROR:
      return "Write error";
    case ESP32OTAPull::JSON_PROBLEM:
      return "Invalid JSON";
    case ESP32OTAPull::OTA_UPDATE_FAIL:
      return "Update fail (no OTA partition?)";
    default:
      if (code > 0)
        return "Unexpected HTTP response code";
      break;
  }
  return "Unknown error";
}

void processTestResult(String testName, String value, String permissible, String range, String location, String unit) {
  // if conducting test via BLE, send the result back to the app
  Serial.println("Processing test result...");
  Serial.println("Test Conducted Via BLE: " + String(bleConductTest));

  String testId = testName + "_" + String(rtc.now().unixtime());
  if (bleConductTest) {
    testId = bleTestId;
    String testStatusMsg = "TEST_STATUS,1," + testName + "," + value + "," + testId;

    sendBleMessage(testStatusMsg);
  }
  writeDataToSD(testName, value, permissible, range, location, unit, testId);
}

void writeDataToSD(String testName, String value, String permissible, String range, String location, String unit, String testId) {
  String dateTime = "";
  DateTime now = rtc.now();
  dateTime = String(now.hour()) + ":" + (now.minute() < 10 ? "0" + String(now.minute()) : String(now.minute()));

  int serial = 0;
  serial = readCSVFile();
  // Prepare new data row in CSV format
  // String newRow = serial + "," + testName + "," + value + "," + dateTime + "," + permissible + "," + range + "," + location + "," + unit + "\n";

  // Open the existing file and read its content
  File file = SD.open(FILE_NAME, FILE_READ);
  String existingData = "";
  if (file) {
    while (file.available()) {
      existingData += char(file.read());
    }
    file.close();
  } else {
    Serial.println("File does not exist or failed to open. Creating a new file.");
  }

  // Reopen the file for writing and write the new data followed by the existing content
  file = SD.open(FILE_NAME, FILE_WRITE);
  if (file) {
    file.printf("%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%02d/%02d/%04d,%02d:%02d,%01d\n",
                serial,
                testName,
                value,
                dateTime,
                permissible,
                range,
                location,
                unit,
                // BLE Vars
                testId.c_str(),
                latestTestLat.c_str(),
                latestTestLng.c_str(),
                latestTestAddressId.c_str(),
                now.day(),
                now.month(),
                now.year(),
                now.hour(),
                now.minute(),
                TestSyncStatus::TEST_LOCAL);
    file.print(existingData);  // Append the old data
    file.close();
    Serial.println("Data prepended successfully to CSV.");
  } else {
    Serial.println("Failed to open file for writing.");
  }
}

int readCSVFile() {
  Serial.println("in");
  fileSize = 0;
  File file = SD.open(FILE_NAME);
  if (!file) {
    Serial.println("Failed to open file");
    return 0;
  }
  flushHistoryRecords();
  // Read and display the file content
  while (file.available()) {
    String line = file.readStringUntil('\n');
    addHistoryRecord(line);
    fileSize++;
  }
  file.close();
  return fileSize;
}

History parseData(String data) {
  History history;

  // Parse each part of the string using ',' as the separator
  int index = 0;
  int start = 0;
  int end = data.indexOf(',');

  // Serial
  history.serial = data.substring(start, end).toInt();

  // Test Name
  start = end + 1;
  end = data.indexOf(',', start);
  history.testName = data.substring(start, end);

  // Value
  start = end + 1;
  end = data.indexOf(',', start);
  history.value = data.substring(start, end);

  // DateTime
  start = end + 1;
  end = data.indexOf(',', start);
  history.dateTime = data.substring(start, end);

  // Permissible
  start = end + 1;
  end = data.indexOf(',', start);
  history.permissible = data.substring(start, end);

  // Range
  start = end + 1;
  end = data.indexOf(',', start);
  history.range = data.substring(start, end);

  // Location
  start = end + 1;
  end = data.indexOf(',', start);
  history.location = data.substring(start, end);

  // Unit
  start = end + 1;
  end = data.indexOf(',', start);
  history.unit = data.substring(start, end);

  return history;
}

// Function to add a new history record to the array
void addHistoryRecord(String data) {
  if (currentRecordCount < MAX_RECORDS) {
    // Parse the data and add it to the history array
    historyRecords[currentRecordCount] = parseData(data);
    currentRecordCount++;
  } else {
    //Serial.println("Maximum record limit reached. Cannot add more records.");
  }
}

void flushHistoryRecords() {
  // Reset the count of current records
  currentRecordCount = 0;

  // Optionally, clear each record in the array
  for (int i = 0; i < MAX_RECORDS; i++) {
    historyRecords[i].serial = 0;
    historyRecords[i].testName = "";
    historyRecords[i].value = "";
    historyRecords[i].dateTime = "";
    historyRecords[i].permissible = "";
    historyRecords[i].range = "";
    historyRecords[i].location = "";
    historyRecords[i].unit = "";
  }

  Serial.println("History records have been flushed.");
}


// -*-*-*-*-*-*-*-*-*-*-*-*-*- BLE Start -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

void splitString(String inputString, String parts[], int &numParts,
                 int maxParts) {
  numParts = 0;
  int startIndex = 0;
  int delimiterIndex = inputString.indexOf(",");

  while (delimiterIndex != -1 && numParts < maxParts) {
    parts[numParts] = inputString.substring(startIndex, delimiterIndex);
    startIndex = delimiterIndex + 1;
    delimiterIndex = inputString.indexOf(",", startIndex);
    numParts++;
  }

  // Handle the last part of the string after the last delimiter
  if (startIndex < inputString.length() && numParts < maxParts) {
    parts[numParts] = inputString.substring(startIndex);
    numParts++;
  }
}
bool isBleAck(String msg) {
  String suffix = "_ACK";
  //  size_t len = ;
  //  size_t suffix_len = ;

  if (msg.length() >= suffix.length() && strcmp(msg.c_str() + msg.length() - suffix.length(), suffix.c_str()) == 0) {
    return true;
  }
  return false;
}

bool isAckResolved = true;
void processAck(String parts[]) {
  isAckResolved = true;

  if (parts[0] == "TEST_STATUS_ACK") {
    // TODO: Yash, Handle this case if you need any special input from app
  }
}

void BLEGoHome() {
  String goHomeAck = "GO_HOME_ACK,1,";
  ackMessage(goHomeAck);
}

void BLEPerformTest() {
  String parts[10];
  int numParts = 0;
  splitString(BLEValue, parts, numParts, 10);

  latestTestAddressId = parts[2];
  latestTestLat = parts[3];
  latestTestLng = parts[4];

  String testAck = "CONDUCT_TEST_ACK," + parts[1] + ",1";
  ackMessage(testAck);

  // TODO: conduct test here
  bleTest = parts[1];
  bleTestId = parts[5];
}

void BLEPerformZeroCalibration() {
  String parts[10];
  int numParts = 0;
  splitString(BLEValue, parts, numParts, 10);

  String testAck = "ZERO_CALIBRATION_ACK," + parts[1] + ",1";
  ackMessage(testAck);

  bleZeroingTest = parts[1];
}

void BLEAction() {
  String BLEparts[20];
  int numParts = 0;
  splitString(BLEValue, BLEparts, numParts, 20);

  if (isBleAck(BLEparts[0])) {
    processAck(BLEparts);
  } else if (BLEparts[0] == "HEALTH") {
    BLEHealth();
  } else if (BLEparts[0] == "USERINFO") {
    // if (BLEparts[1] == "GET") {
    //   BLEGetUserInfo();
    // } else if (BLEparts[1] == "SET") {
    //   BLESetUserInfo();
    // }
  } else if (BLEparts[0] == "SHUTDOWN") {
    // esp shutdown
  } else if (BLEparts[0] == "UPDATE") {
    UPDATEDEVICE();
  } else if (BLEparts[0] == "CONDUCT_TEST") {
    underAppControl = false;
    BLEPerformTest();
  } else if (BLEparts[0] == "ZERO_CALIBRATION") {
    BLEPerformZeroCalibration();
  } else if (BLEparts[0] == "LOCAL_TESTS") {
    BLELocalTests();
  } else if (BLEparts[0] == "APP_CONTROL_START") {
    underAppControl = true;
    String testAck = "APP_CONTROL_START_ACK,1,";
    ackMessage(testAck);
  } else if (BLEparts[0] == "APP_CONTROL_STOP") {
    underAppControl = false;
    dwin.write(home_page_change, 10);
    String testAck = "APP_CONTROL_STOP_ACK,1,";
    ackMessage(testAck);
  } else if (BLEparts[0] == "REAGENT_SELECT") {
    // send ack
    String testAck = "REAGENT_SELECT_ACK,1";
    ackMessage(testAck);
    bleReagent = BLEparts[1];
  } else if (BLEparts[0] == "REAGENT_DROPLETS") {
    String testAck = "REAGENT_DROPLETS_ACK,1";
    ackMessage(testAck);
    bleDroplets = BLEparts[1].toInt();
  }

  // Post test commands
  if (BLEparts[0] == "GO_HOME") {
    BLEGoHome();
  }
  BLEValue = "";
}

void ackMessage(String ack) {
  Serial.println("Writing Ack Message to BLE:" + ack);
  const char *cAck = ack.c_str();
  pCharacteristic->setValue(cAck);
  pCharacteristic->notify();
}

void sendBleMessage(String msg) {
  Serial.println("Writing Message to BLE:" + msg);
  const char *cMsg = msg.c_str();
  pCharacteristic->setValue(cMsg);
  pCharacteristic->notify();

  isAckResolved = false;
}

String getSerialNo() {
  String deviceMacAddress = WiFi.macAddress();
  Serial.println(deviceMacAddress);
  for (int i = 0; i < 20; i++) {
    if (deviceMacAddress.equals(deviceInformationList[i].deviceMacAddress)) {
      return deviceInformationList[i].deviceSerialNumber;
    }
  }
  return "3010";
}

void BLEHealth() {
  //Heath Status
  //ack,serialNo,isSuccess,msg,state, battery,swVersion, hwVersion,pendingTests
  //1. HEALTH_ACK,CA020201,1,,IDLE,48.2,1.1.0,1.2.0,10
  String serialNum = getSerialNo();
  int batteryPercentage = calculateBatteryPercent();
  String healthAck = "HEALTH_ACK," + serialNum + "," + "1," + "," + "IDLE" + "," + batteryPercentage + "," + VERSION + "," + "C012," + "pendingTests";
  ackMessage(healthAck);
}

int syncedUpto = -1;
void BLELocalTests() {
  Serial.println("BLELocalTests");
  // read all tests from file
  // send to app
  File file = SD.open(FILE_NAME);
  if (!file) {
    Serial.println("Failed to open file for reading");
    delay(5);
    // pcf8575.digitalWrite(P15, HIGH);
    return;
  }

  String line = "";
  u_int8_t MAX_SIZE = 50;
  u_int8_t sampleCount = 0;
  file.seek(0, SeekEnd);
  String bleContent = "TESTS_DATA_START,1,\n";

  // TODO: TESTING
  sendBleMessage(bleContent);

  // append in bleContent

  // while (file.position() > 0 && sampleCount < MAX_SIZE) {
  //   char c;
  //   do {
  //     Serial.print("Reading file. Position: ");
  //     Serial.println(file.position());
  //     Serial.println(line);
  //     if (file.position() == 1) {
  //       // If we're at the beginning of the file, read the first character
  //       c = file.read();
  //       line = c + line;
  //       break;
  //     }
  //     // file.seek(file.position() - 1);
  //     c = file.read();
  //     file.seek(file.position() - 2);
  //     if (c == '\n') {
  //       break;
  //     }
  //     line = c + line;
  //   } while (true);

  //   // if (file.position() != 0) {
  //   //   // If not at the beginning of the file, move one character back
  //   //   file.seek(file.position() - 1);
  //   // }

  //   // Check for condition at the end of the file
  //   if (lastLine.length() > 0) {
  //     if (line.charAt(line.length() - 1) == '1') {
  //       break;
  //     } else {
  //       // bleContent += line + "\n";
  //       sendBleMessage(line);
  //       sampleCount++;
  //     }
  //   }
  //   lastLine = line;
  //   line = "";
  // }

  // Move to the end of the file
  // file.seek(0);
  while (file.position() > 1 && sampleCount < MAX_SIZE) {
    char c;
    do {
      Serial.print("Reading file. Position: ");
      Serial.println(file.position());
      Serial.println(line);
      // Read a character
      c = file.read();
      if (file.position() == 1) {
        // If we're at the beginning of the file, read the first character
        line = c + line;
        break;
      }
      if (c == '\n') {
        file.seek(file.position() - 2);
        break;
      }
      // Move the file pointer backward by 2 characters
      file.seek(file.position() - 2);
      line = c + line;
    } while (true);

    // Check for condition at the end of the file
    if (line.length() > 0) {
      if (line.charAt(line.length() - 1) == '1') {
        break;
      } else {
        sendBleMessage(line);
        sampleCount++;
      }
    }
    line = "";
  }

  syncedUpto = file.position();

  // TESTS_DATA_ENDED,1, + no of tests`
  String testEnded = "TESTS_DATA_END,1," + String(sampleCount);
  sendBleMessage(testEnded);

  // Close the file
  file.close();

  Serial.println("BLELocalTests Completed");
}

void updateSyncedTestStatus() {
  if (syncedUpto == -1) {
    Serial.println("No tests to sync");
    return;
  }

  File file = SD.open(FILE_NAME, FILE_WRITE);
  String line;

  file.seek(syncedUpto);
  while (file.available()) {
    line = file.readStringUntil('\n');
    line.setCharAt(line.length() - 1, '1');
  }

  file.close();

  syncedUpto = -1;
}
// -*-*-*-*-*-*-*-*-*-*-*-*-*- BLE End -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
