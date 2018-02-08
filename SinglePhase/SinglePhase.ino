#include "ESP8266WiFi.h"
#include <SPI.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 3
Adafruit_SSD1306 display(OLED_RESET);
#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
#if (SSD1306_LCDHEIGHT != 32)
  #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#include "ATM90E26_SPI.h"
#include "EnergyIC.h"
ATM90E26_SPI atm90e26;
EnergyIC eic(atm90e26);

void setup()
{
  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();

  EnergyIC_CalibrationSettings settings;
  settings.referenceVoltage_volts = 120;
  settings.baseCurrent_amps = 7;
  settings.currentTransformerRange_amps = 100;
  settings.currentTransformerOutput_milliamps = 50;
  settings.currentBurdenResistor_ohms = 12;
  settings.voltageTransformerOutput_volts = 13.69;
  settings.voltageDividerResistor1_ohms = 100000;
  settings.voltageDividerResistor2_ohms = 1000;

  settings.actualMainsVoltage_volts = 119.9;
  settings.reportedVoltage_volts = 113.97;
  settings.actualMainsCurrent_amps = 7.09;
  settings.reportedCurrent_amps = 7.7;

  eic.Initialize(settings);
}

void loop()
{
  double lineVoltage = eic.GetLineVoltage();
  double lineCurrent = eic.GetLineCurrent();
  double activePower = eic.GetActivePower();
  double powerFactor = eic.GetPowerFactor();
  double importedEnergy = eic.GetImportEnergy() * 1000;

  displayReading("Voltage: ", lineVoltage);
  displayReading("Current: ", lineCurrent);
  displayReading("Active power: ", activePower);
  displayReading("Power factor: ", powerFactor);
  displayReading("Energy: ", importedEnergy);
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