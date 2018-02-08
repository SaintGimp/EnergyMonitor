 /* ATM90E26 Energy Monitor Demo Application

    The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
 
 #include <SPI.h>
 #include "EnergyIC.h"
 
 unsigned short lowWord(unsigned long value)
 {
   return (short)value;
 }

 unsigned short highWord(unsigned long value)
 {
   return (unsigned short)((value >> 16) & 0xFFFF);
 }

void EnergyIC::Initialize(EnergyIC_CalibrationSettings settings)
{
  atm90e26.Initialize();

	atm90e26.Communicate(0, SoftReset, 0x789A); // Perform soft reset
	atm90e26.Communicate(0, FuncEn, 0x0030); // Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
	atm90e26.Communicate(0, SagTh, 0x1F2F); // Voltage sag threshhold

	//Set metering calibration values
  double milliampsPerAmp = settings.currentTransformerOutput_milliamps / settings.currentTransformerRange_amps;
  double expectedCurrentSampleVoltage_millivolts = milliampsPerAmp * settings.baseCurrent_amps * settings.currentBurdenResistor_ohms;
  Serial.print("expectedCurrentSampleVoltage_millivolts = ");
  Serial.println(expectedCurrentSampleVoltage_millivolts);

  double expectedVoltageSampleVoltage_millivolts = 1000 * settings.voltageTransformerOutput_volts * (settings.voltageDividerResistor2_ohms / (settings.voltageDividerResistor1_ohms + settings.voltageDividerResistor2_ohms));
  Serial.print("expectedVoltageSampleVoltage_millivolts = ");
  Serial.println(expectedVoltageSampleVoltage_millivolts);

  int currentAmplifierGain = 1;
  int pulseConstant = 60000;
  unsigned long plConstant = 838860800 * ((currentAmplifierGain * expectedCurrentSampleVoltage_millivolts * expectedVoltageSampleVoltage_millivolts) / (pulseConstant * settings.referenceVoltage_volts * settings.baseCurrent_amps));
  // TODO: There was something about making this a multiple of 4?
  Serial.print("plConstant = ");
  Serial.println(plConstant, HEX);

  unsigned short startupPowerThreshold = 93.2067556 * currentAmplifierGain * expectedCurrentSampleVoltage_millivolts * expectedVoltageSampleVoltage_millivolts * 0.004;
  Serial.print("startupPowerThreshold = ");
  Serial.println(startupPowerThreshold, HEX);

  double expectedEnergy = 14.1417;
  double actualEnergy = 11.70;
  double e = -((expectedEnergy - actualEnergy) / expectedEnergy);
  double ratio = -e / (1 + e);
  Serial.print("ratio = ");
  Serial.println(ratio, HEX);
  unsigned short lGain;
  if (ratio > 0)
  {
    lGain = ratio * pow(2, 15);
  }
  else
  {
    lGain = pow(2, 16) + (ratio * pow(2, 15));
  }
  if (actualEnergy == 0)
  {
    lGain = 0;
  }
  Serial.print("lGain = ");
  Serial.println(lGain, HEX);

  atm90e26.Communicate(0, CalStart, 0x5678); // Metering calibration startup command. Register 21 to 2B need to be set
  atm90e26.Communicate(0, PLconstH, highWord(plConstant)); // PL Constant MSB
  atm90e26.Communicate(0, PLconstL, lowWord(plConstant)); // PL Constant LSB
  atm90e26.Communicate(0, Lgain, lGain); // Line calibration gain (to correct energy metering)
  atm90e26.Communicate(0, Lphi, 0x0000); // Line calibration angle
  atm90e26.Communicate(0, PStartTh, startupPowerThreshold); // Active Startup Power Threshold
  atm90e26.Communicate(0, PNolTh, 0x0000); // Active No-Load Power Threshold
  atm90e26.Communicate(0, QStartTh, startupPowerThreshold); // Reactive Startup Power Threshold
  atm90e26.Communicate(0, QNolTh, 0x0000); // Reactive No-Load Power Threshold
  atm90e26.Communicate(0, MMode, 0x9422); // Metering Mode Configuration. All defaults. See pg 31 of datasheet.
  atm90e26.Communicate(0, CSOne, atm90e26.Communicate(1, CSOne, 0x0000)); //Write CSOne, as self calculated
  
  Serial.print("Checksum 1:");
  Serial.println(atm90e26.Communicate(1, CSOne, 0x0000), HEX); // Checksum 1. Needs to be calculated based off the above values.

  // Set measurement calibration values
  
  atm90e26.Communicate(0, AdjStart, 0x5678); // Measurement calibration startup command, registers 31-3A
  if (settings.reportedVoltage_volts > 0)
  {
    unsigned long voltageRmsGain = (26400 * settings.actualMainsVoltage_volts) / settings.reportedVoltage_volts;
    atm90e26.Communicate(0, Ugain, voltageRmsGain);
  }
  if (settings.reportedCurrent_amps > 0)
  {
    unsigned long currentRmsGain = (31251 * settings.actualMainsCurrent_amps) / settings.reportedCurrent_amps;
    atm90e26.Communicate(0, IgainL, currentRmsGain);
  }
  atm90e26.Communicate(0, Uoffset, 0x0000);  // Voltage offset
  atm90e26.Communicate(0, IoffsetL, 0x0000); // L line current offset
  atm90e26.Communicate(0, PoffsetL, 0x0000); // L line active power offset
  atm90e26.Communicate(0, QoffsetL, 0x0000); // L line reactive power offset
  atm90e26.Communicate(0, CSTwo, atm90e26.Communicate(1,CSTwo,0x0000)); // Write CSTwo, as self calculated
  
  Serial.print("Checksum 2:");
  Serial.println(atm90e26.Communicate(1,CSTwo,0x0000),HEX);    //Checksum 2. Needs to be calculated based off the above values.
  
  atm90e26.Communicate(0,CalStart,0x8765); //Checks correctness of 21-2B registers and starts normal metering if ok
  atm90e26.Communicate(0,AdjStart,0x8765); //Checks correctness of 31-3A registers and starts normal measurement  if ok

	unsigned short systemstatus;
  systemstatus = GetSysStatus();
  
  if (systemstatus&0xC000){
    //checksum 1 error
    Serial.println("Checksum 1 Error!!");
  }
  if (systemstatus&0x3000){
    //checksum 2 error
    Serial.println("Checksum 2 Error!!");
  }
}

double EnergyIC::GetLineVoltage(){
	unsigned short voltage=atm90e26.Communicate(1,Urms,0xFFFF);
	return (double)voltage/100;
}

unsigned short EnergyIC::GetMeterStatus(){
  return atm90e26.Communicate(1,EnStatus,0xFFFF);
}

double EnergyIC::GetLineCurrent(){
	unsigned short current=atm90e26.Communicate(1,Irms,0xFFFF);
	return (double)current/1000;
}

double EnergyIC::GetActivePower(){
	short int apower= (short int)atm90e26.Communicate(1,Pmean,0xFFFF); //Complement, MSB is signed bit
	return (double)apower;
}

double EnergyIC::GetFrequency(){
	unsigned short freq=atm90e26.Communicate(1,Freq,0xFFFF);
	return (double)freq/100;
}

double EnergyIC::GetPowerFactor(){
	short int pf= (short int)atm90e26.Communicate(1,PowerF,0xFFFF); //MSB is signed bit
	//if negative
	if(pf&0x8000){
		pf=(pf&0x7FFF)*-1;
	}
	return (double)pf/1000;
}

double EnergyIC::GetImportEnergy(){
	//Register is cleared after reading
	unsigned short ienergy=atm90e26.Communicate(1,APenergy,0xFFFF);
	return (double)ienergy / 600000; //response is in "0.1 pulse", returns kWh if PL constant set to 1000imp/kWh
}

double EnergyIC::GetExportEnergy(){
	//Register is cleared after reading
	unsigned short eenergy=atm90e26.Communicate(1,ANenergy,0xFFFF);
	return (double)eenergy*0.0001; //returns kWh if PL constant set to 1000imp/kWh
}

unsigned short EnergyIC::GetSysStatus(){
	return atm90e26.Communicate(1,SysStatus,0xFFFF);
}










