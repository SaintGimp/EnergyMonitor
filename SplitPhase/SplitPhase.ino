#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <Wire.h>

#include <JeVe_EasyOTA.h>
EasyOTA OTA;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 3
Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);

#include "secrets.h"

#include <ArduinoJson.h>

#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
void updateCallback();
Task updateTask(1000, TASK_FOREVER, &updateCallback);
Scheduler runner;

#include "ATM90E26_SPI.h"
#include "ATM90E26.h"
ATM90E26_SPI atm90e26_spi_1(15);
ATM90E26 atm90e26_1(atm90e26_spi_1);
ATM90E26_SPI atm90e26_spi_2(0);
ATM90E26 atm90e26_2(atm90e26_spi_2);

#define CURRENT_MONITOR_ADDRESS 0x08
#define SUPPLY_VOLTAGE_REGISTER 0
#define ICAL0_REGISTER 2
#define ICAL1_REGISTER 4
#define CHANNEL0_REGISTER 6
#define CHANNEL1_REGISTER 8


struct EnergyReadings
{
  double lineVoltage;
  double lineCurrent;
  double activePower;
  double powerFactor;
  double importedEnergy;
  double exportedEnergy;
  double auxCurrent;
};

void setup()
{
  Serial.begin(115200);
  Serial.println();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("OTA set up.\nSetting up monitors...");
  display.display();

  initializeEnergyMonitor();
  initializeCurrentMonitor();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Monitors set up.");
  display.display();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting to wifi...");
  display.display();

  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(wifiSSID, wifiPassword);
  wl_status_t status;
  while ((status = (wl_status_t)WiFi.waitForConnectResult()) != WL_CONNECTED)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Connecting to wifi...\n");
    display.print(wl_status_to_string(status));
    display.display();
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Wifi connected.\nSetting up OTA...");
  display.display();

  OTA.onMessage([](char *message, int line) {
    Serial.println(message);
  });
  OTA.setup((char*)wifiSSID, (char*)wifiPassword, (char*)"EnergyMonitor");

  runner.init();
  runner.addTask(updateTask);
  updateTask.enable();

  // Zero out energy counters
  atm90e26_1.GetImportEnergy();
  atm90e26_2.GetImportEnergy();
  atm90e26_1.GetExportEnergy();
  atm90e26_2.GetExportEnergy();
}

void loop()
{
  OTA.loop();
  runner.execute();
}

const char* wl_status_to_string(wl_status_t status) {
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
  }
  
  return "UNKNOWN";
}

void updateCallback()
{
  EnergyReadings readings_1;
  readings_1.lineVoltage = atm90e26_1.GetLineVoltage();
  readings_1.lineCurrent = atm90e26_1.GetLineCurrent();
  readings_1.activePower = atm90e26_1.GetActivePower();
  readings_1.powerFactor = atm90e26_1.GetPowerFactor();
  readings_1.importedEnergy = atm90e26_1.GetImportEnergy() * 1000;
  readings_1.exportedEnergy = atm90e26_1.GetExportEnergy() * 1000;
  readings_1.auxCurrent = ReadCurrent(CHANNEL0_REGISTER);
  
  EnergyReadings readings_2;
  readings_2.lineVoltage = atm90e26_2.GetLineVoltage();
  readings_2.lineCurrent = atm90e26_2.GetLineCurrent();
  readings_2.activePower = atm90e26_2.GetActivePower();
  readings_2.powerFactor = atm90e26_2.GetPowerFactor();
  readings_2.importedEnergy = atm90e26_2.GetImportEnergy() * 1000;
  readings_2.exportedEnergy = atm90e26_2.GetExportEnergy() * 1000;
  readings_2.auxCurrent = ReadCurrent(CHANNEL1_REGISTER);
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.display();

  uploadData(readings_1, readings_2);
}

float ReadCurrent(uint registerAddress)
{
  // Point to the value we want to read
  Wire.beginTransmission(CURRENT_MONITOR_ADDRESS);
  Wire.write(registerAddress);
  Wire.endTransmission();

  // Request the value for the channel
  if (Wire.requestFrom(CURRENT_MONITOR_ADDRESS, 2) < 2)
  {
    //I2C_ClearBus();
    //Wire.begin();
    Serial.println("I2C failure");
    return -1;
  }

  float value = (uint)(Wire.read() | Wire.read() << 8) / 1000.0;

  return value;
}

void initializeEnergyMonitor()
{
  ATM90E26_CalibrationSettings settings;
  settings.referenceVoltage_volts = 120;
  settings.currentTransformerRange_amps = 100;
  settings.currentTransformerOutput_milliamps = 50;
  settings.currentBurdenResistor_ohms = 12;
  settings.voltageTransformerOutput_volts = 12.5;
  settings.voltageDividerResistor1_ohms = 100000;
  settings.voltageDividerResistor2_ohms = 1000;

  settings.actualMainsVoltage_volts = 120.9;
  settings.reportedVoltage_volts = 103.7;
  settings.actualMainsCurrent_amps = 7.09;
  settings.reportedCurrent_amps = 7.7;
  settings.actualEnergy_watthours = 13.9;
  settings.reportedEnergy_watthours = 11.42;

  atm90e26_1.Initialize(settings);

  settings.referenceVoltage_volts = 120;
  settings.currentTransformerRange_amps = 100;
  settings.currentTransformerOutput_milliamps = 50;
  settings.currentBurdenResistor_ohms = 12;
  settings.voltageTransformerOutput_volts = 12.5;
  settings.voltageDividerResistor1_ohms = 100000;
  settings.voltageDividerResistor2_ohms = 1000;

  settings.actualMainsVoltage_volts = 120.8;
  settings.reportedVoltage_volts = 103.65;
  settings.actualMainsCurrent_amps = 7.20;
  settings.reportedCurrent_amps = 7.83;
  settings.actualEnergy_watthours = 13.9;
  settings.reportedEnergy_watthours = 11.42;

  atm90e26_2.Initialize(settings);

  // TODO: do we need to wait until the device stabilizes?
  // Do we need to zero out energy counter?
  //calibrateEnergyGain(60);
}

void initializeCurrentMonitor()
{
  // Values are transferred as uints (float value * 1000)
  uint supplyVoltage = 3273;  // 3300 nominal, this is the measured value
  uint calibration0 = 42550;
  uint calibration1 = 42550;

  Wire.beginTransmission(CURRENT_MONITOR_ADDRESS);
  Wire.write(SUPPLY_VOLTAGE_REGISTER);
  Wire.write(lowByte(supplyVoltage));
  Wire.write(highByte(supplyVoltage));
  Wire.endTransmission();

  Wire.beginTransmission(CURRENT_MONITOR_ADDRESS);
  Wire.write(ICAL0_REGISTER);
  Wire.write(lowByte(calibration0));
  Wire.write(highByte(calibration0));
  Wire.endTransmission();

  Wire.beginTransmission(CURRENT_MONITOR_ADDRESS);
  Wire.write(ICAL1_REGISTER);
  Wire.write(lowByte(calibration1));
  Wire.write(highByte(calibration1));
  Wire.endTransmission();
}

void uploadData(EnergyReadings& readings_1, EnergyReadings& readings_2)
{
  int httpCode = 0;

  if (WiFi.status() != WL_CONNECTED)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Not connected!");
    display.display();

    delay(1000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.display();

    return;
  }

  BearSSL::WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  https.begin(client, logstashUrl);
  https.addHeader("Content-Type", "application/json");
  https.setAuthorization(logstashUserName, logstashPassword);

  const size_t bufferSize = JSON_OBJECT_SIZE(12);
  StaticJsonDocument<bufferSize> jsonDocument;

  JsonObject root = jsonDocument.to<JsonObject>();
  root["lineVoltage1"] = readings_1.lineVoltage;
  root["lineCurrent1"] = readings_1.lineCurrent;
  root["activePower1"] = readings_1.activePower;
  root["powerFactor1"] = readings_1.powerFactor;
  root["importedEnergy1"] = readings_1.importedEnergy;
  if (readings_1.auxCurrent >= 0)
  {
    root["auxCurrent1"] = readings_1.auxCurrent;
  }
  
  root["lineVoltage2"] = readings_2.lineVoltage;
  root["lineCurrent2"] = readings_2.lineCurrent;
  root["activePower2"] = readings_2.activePower;
  root["powerFactor2"] = readings_2.powerFactor;
  root["importedEnergy2"] = readings_2.importedEnergy;
  if (readings_2.auxCurrent >= 0)
  {
    root["auxCurrent2"] = readings_2.auxCurrent;
  }

  String payload;
  serializeJson(root, payload);

  httpCode = https.POST(payload);

  if (httpCode < 200 || httpCode >= 300)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Status code ");
    display.println(httpCode);
    display.display();

    delay(1000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.display();
  }
}

void calibrateEnergyGain(unsigned long calibrationLengthInSeconds)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.display();
  
  double actualEnergy_watthours = 0;
  double reportedEnery_watthours = 0;

  // Clear the energy counter
  atm90e26_1.GetImportEnergy();

  unsigned long beginTime = millis();
  for (int i = 0; i < calibrationLengthInSeconds; i++)
  {
    // Ensure we're reading every 1000 milliseconds,
    // correcting for time spent reading and updating
    unsigned long nextReadTime = beginTime + (1000 * (i + 1));
    unsigned long sleepLength = max(nextReadTime - millis(), (unsigned long)0);
    delay(sleepLength);

    double lineVoltage = atm90e26_1.GetLineVoltage();
    double lineCurrent = atm90e26_1.GetLineCurrent();

    actualEnergy_watthours += (lineVoltage * lineCurrent) / 3600.0;
    reportedEnery_watthours += atm90e26_1.GetImportEnergy() * 1000.0;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Calibrating... ");
    display.println(calibrationLengthInSeconds - i);
    display.display();
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Actual wh: ");
  display.println(actualEnergy_watthours);
  display.print("Reported wh: ");
  display.println(reportedEnery_watthours);
  display.display();

  delay(60 * 1000);
}

int I2C_ClearBus() {
  // http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html

#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}
