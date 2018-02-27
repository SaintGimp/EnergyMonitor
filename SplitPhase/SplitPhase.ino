#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#include <JeVe_EasyOTA.h>
EasyOTA OTA;

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 3
Adafruit_SSD1306 display(OLED_RESET);
#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
#if (SSD1306_LCDHEIGHT != 32)
  #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

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

struct EnergyReadings
{
  double lineVoltage;
  double lineCurrent;
  double activePower;
  double powerFactor;
  double importedEnergy;
};

void setup()
{
  Serial.begin(115200);
  Serial.println();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);

  initializeEnergyMonitor();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Connecting to wifi...");
  display.display();

  WiFi.begin(wifiSSID, wifiPassword);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  OTA.onMessage([](char *message, int line) {
    Serial.println(message);
  });
  OTA.setup((char*)wifiSSID, (char*)wifiPassword, "EnergyMonitor");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connected.");
  display.display();

  runner.init();
  runner.addTask(updateTask);
  updateTask.enable();

  // Zero out energy counters
  atm90e26_1.GetImportEnergy();
  atm90e26_2.GetImportEnergy();
}

void loop()
{
  OTA.loop();
  runner.execute();
}

void updateCallback()
{
  EnergyReadings readings_1;
  readings_1.lineVoltage = atm90e26_1.GetLineVoltage();
  readings_1.lineCurrent = atm90e26_1.GetLineCurrent();
  readings_1.activePower = atm90e26_1.GetActivePower();
  readings_1.powerFactor = atm90e26_1.GetPowerFactor();
  readings_1.importedEnergy = atm90e26_1.GetImportEnergy() * 1000;
  EnergyReadings readings_2;
  readings_2.lineVoltage = atm90e26_2.GetLineVoltage();
  readings_2.lineCurrent = atm90e26_2.GetLineCurrent();
  readings_2.activePower = atm90e26_2.GetActivePower();
  readings_2.powerFactor = atm90e26_2.GetPowerFactor();
  readings_2.importedEnergy = atm90e26_2.GetImportEnergy() * 1000;

  display.clearDisplay();
  display.setCursor(0, 0);

  display.print("V: ");
  display.print(readings_1.lineVoltage);
  display.print(" C: ");
  display.println(readings_1.lineCurrent);
  display.print("P: ");
  display.print(readings_1.activePower);
  display.print(" PF: ");
  display.println(readings_1.powerFactor);
  display.print("V: ");
  display.print(readings_2.lineVoltage);
  display.print(" C: ");
  display.println(readings_2.lineCurrent);
  display.print("P: ");
  display.print(readings_2.activePower);
  display.print(" PF: ");
  display.print(readings_2.powerFactor);
  
  display.display();

  uploadData(readings_1, readings_2);
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

void uploadData(EnergyReadings& readings_1, EnergyReadings& readings_2)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Not connected!");
    display.display();
  }

  HTTPClient http;
  http.begin(logstashUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader(keyHeader, logstashKey);

  const size_t bufferSize = JSON_OBJECT_SIZE(10);
  StaticJsonBuffer<bufferSize> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  root["lineVoltage1"] = readings_1.lineVoltage;
  root["lineCurrent1"] = readings_1.lineCurrent;
  root["activePower1"] = readings_1.activePower;
  root["powerFactor1"] = readings_1.powerFactor;
  root["importedEnergy1"] = readings_1.importedEnergy;
  root["lineVoltage2"] = readings_2.lineVoltage;
  root["lineCurrent2"] = readings_2.lineCurrent;
  root["activePower2"] = readings_2.activePower;
  root["powerFactor2"] = readings_2.powerFactor;
  root["importedEnergy2"] = readings_2.importedEnergy;

  String payload;
  root.printTo(payload);

  int httpCode = http.POST(payload);

  if (httpCode < 200 || httpCode >= 300)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Status code ");
    display.println(httpCode);
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
