#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

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

#include "ATM90E26_SPI.h"
#include "ATM90E26.h"
ATM90E26_SPI atm90e26_spi(15);
ATM90E26 atm90e26(atm90e26_spi);

void setup()
{
  Serial.begin(115200);
  Serial.println();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);

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

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();

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

  atm90e26.Initialize(settings);
  // TODO: do we need to wait until the device stabilizes?
  // Do we need to zero out energy counter?
  //calibrateEnergyGain(60);
}

void loop()
{
  double lineVoltage = atm90e26.GetLineVoltage();
  double lineCurrent = atm90e26.GetLineCurrent();
  double activePower = atm90e26.GetActivePower();
  double powerFactor = atm90e26.GetPowerFactor();
  double importedEnergy = atm90e26.GetImportEnergy() * 1000;

  displayReading("Voltage: ", lineVoltage);
  displayReading("Current: ", lineCurrent);
  displayReading("Active power: ", activePower);
  displayReading("Power factor: ", powerFactor);
  displayReading("Energy: ", importedEnergy);

  uploadData(lineVoltage, lineCurrent, activePower, powerFactor, importedEnergy);
}

void displayReading(const char* label, double value)
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(label);
  display.println(value);
  display.display();
  delay(2000);
}

void uploadData(double lineVoltage, double lineCurrent, double activePower, double powerFactor, double importedEnergy)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Not connected, can't upload!");
  }

  HTTPClient http;
  http.begin(logstashUrl);
  http.addHeader("Content-Type", "application/json");
  http.addHeader(keyHeader, logstashKey);

  const size_t bufferSize = JSON_OBJECT_SIZE(5);
  StaticJsonBuffer<bufferSize> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  root["lineVoltage"] = lineVoltage;
  root["lineCurrent"] = lineCurrent;
  root["activePower"] = activePower;
  root["powerFactor"] = powerFactor;
  root["importedEnergy"] = importedEnergy;

  String payload;
  root.printTo(payload);

  int httpCode = http.POST(payload);

  if (httpCode < 200 || httpCode >= 300)
  {
    Serial.println("Failed to upload, status code " + httpCode);
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
  atm90e26.GetImportEnergy();

  unsigned long beginTime = millis();
  for (int i = 0; i < calibrationLengthInSeconds; i++)
  {
    // Ensure we're reading every 1000 milliseconds,
    // correcting for time spent reading and updating
    unsigned long nextReadTime = beginTime + (1000 * (i + 1));
    unsigned long sleepLength = max(nextReadTime - millis(), (unsigned long)0);
    delay(sleepLength);

    double lineVoltage = atm90e26.GetLineVoltage();
    double lineCurrent = atm90e26.GetLineCurrent();

    actualEnergy_watthours += (lineVoltage * lineCurrent) / 3600.0;
    reportedEnery_watthours += atm90e26.GetImportEnergy() * 1000.0;

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