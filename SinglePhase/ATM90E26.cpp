 /* ATM90E26 Energy Monitor Demo Application

    The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
 
#include "ATM90E26.h"
 
unsigned short lowWord(unsigned long value)
{
  return (short)value;
}

unsigned short highWord(unsigned long value)
{
  return (unsigned short)((value >> 16) & 0xFFFF);
}

void ATM90E26::Initialize(ATM90E26_CalibrationSettings settings)
{
  transport.Initialize();

	transport.Communicate(0, SoftReset, 0x789A); // Perform soft reset
	transport.Communicate(0, FuncEn, 0x0030); // Voltage sag irq=1, report on warnout pin=1, energy dir change irq=0
	transport.Communicate(0, SagTh, 0x1F2F); // Voltage sag threshhold

	//Set metering calibration values
  double referenceCurrent_amps = 10;
  double milliampsPerAmp = settings.currentTransformerOutput_milliamps / settings.currentTransformerRange_amps;
  double expectedCurrentSampleVoltage_millivolts = milliampsPerAmp * referenceCurrent_amps * settings.currentBurdenResistor_ohms;
  double expectedVoltageSampleVoltage_millivolts = 1000 * settings.voltageTransformerOutput_volts * (settings.voltageDividerResistor2_ohms / (settings.voltageDividerResistor1_ohms + settings.voltageDividerResistor2_ohms));

  int currentAmplifierGain = 1;
  unsigned long plConstant = 838860800 * ((currentAmplifierGain * expectedCurrentSampleVoltage_millivolts * expectedVoltageSampleVoltage_millivolts) / (PULSE_CONSTANT * settings.referenceVoltage_volts * referenceCurrent_amps));
  // TODO: There was something in the datasheet about making this a multiple of 4?

  unsigned short startupPowerThreshold = 93.2067556 * currentAmplifierGain * expectedCurrentSampleVoltage_millivolts * expectedVoltageSampleVoltage_millivolts * 0.004;

  double e = -((settings.actualEnergy_watthours - settings.reportedEnergy_watthours) / settings.actualEnergy_watthours);
  double ratio = -e / (1 + e);
  unsigned short lGain;
  if (ratio > 0)
  {
    lGain = ratio * pow(2, 15);
  }
  else
  {
    lGain = pow(2, 16) + (ratio * pow(2, 15));
  }
  if (settings.reportedEnergy_watthours == 0)
  {
    lGain = 0;
  }

  transport.Communicate(0, CalStart, 0x5678); // Metering calibration startup command. Register 21 to 2B need to be set
  transport.Communicate(0, PLconstH, highWord(plConstant)); // PL Constant MSB
  transport.Communicate(0, PLconstL, lowWord(plConstant)); // PL Constant LSB
  transport.Communicate(0, Lgain, lGain); // Line calibration gain (to correct energy metering)
  transport.Communicate(0, Lphi, 0x0000); // Line calibration angle
  transport.Communicate(0, PStartTh, startupPowerThreshold); // Active Startup Power Threshold
  transport.Communicate(0, PNolTh, 0x0000); // Active No-Load Power Threshold
  transport.Communicate(0, QStartTh, startupPowerThreshold); // Reactive Startup Power Threshold
  transport.Communicate(0, QNolTh, 0x0000); // Reactive No-Load Power Threshold
  transport.Communicate(0, MMode, 0x9422); // Metering Mode Configuration. All defaults. See pg 31 of datasheet.
  transport.Communicate(0, CSOne, transport.Communicate(1, CSOne, 0x0000)); //Write CSOne, as self calculated
  
  // Set measurement calibration values
  
  transport.Communicate(0, AdjStart, 0x5678); // Measurement calibration startup command, registers 31-3A
  if (settings.reportedVoltage_volts > 0)
  {
    unsigned long voltageRmsGain = (26400 * settings.actualMainsVoltage_volts) / settings.reportedVoltage_volts;
    transport.Communicate(0, Ugain, voltageRmsGain);
  }
  if (settings.reportedCurrent_amps > 0)
  {
    unsigned long currentRmsGain = (31251 * settings.actualMainsCurrent_amps) / settings.reportedCurrent_amps;
    transport.Communicate(0, IgainL, currentRmsGain);
  }
  transport.Communicate(0, Uoffset, 0x0000);  // Voltage offset
  transport.Communicate(0, IoffsetL, 0x0000); // L line current offset
  transport.Communicate(0, PoffsetL, 0x0000); // L line active power offset
  transport.Communicate(0, QoffsetL, 0x0000); // L line reactive power offset
  transport.Communicate(0, CSTwo, transport.Communicate(1,CSTwo,0x0000)); // Write CSTwo, as self calculated
    
  transport.Communicate(0,CalStart,0x8765); //Checks correctness of 21-2B registers and starts normal metering if ok
  transport.Communicate(0,AdjStart,0x8765); //Checks correctness of 31-3A registers and starts normal measurement  if ok

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

double ATM90E26::GetLineVoltage(){
	unsigned short voltage=transport.Communicate(1,Urms,0xFFFF);
	return (double)voltage/100;
}

unsigned short ATM90E26::GetMeterStatus(){
  return transport.Communicate(1,EnStatus,0xFFFF);
}

double ATM90E26::GetLineCurrent(){
	unsigned short current=transport.Communicate(1,Irms,0xFFFF);
	return (double)current/1000;
}

double ATM90E26::GetActivePower(){
	short int apower= (short int)transport.Communicate(1,Pmean,0xFFFF); //Complement, MSB is signed bit
	return (double)apower;
}

double ATM90E26::GetFrequency(){
	unsigned short freq=transport.Communicate(1,Freq,0xFFFF);
	return (double)freq/100;
}

double ATM90E26::GetPowerFactor(){
	short int pf= (short int)transport.Communicate(1,PowerF,0xFFFF); //MSB is signed bit
	//if negative
	if (pf&0x8000) {
		pf = (pf&0x7FFF) * -1;
	}
	return (double)pf / 1000;
}

double ATM90E26::GetImportEnergy(){
	// Register is cleared after reading
  // response is in "0.1 pulse"
	unsigned short ienergy = transport.Communicate(1, APenergy, 0xFFFF);
	return (double)ienergy / (PULSE_CONSTANT * 10); // return value is in kWh 
}

double ATM90E26::GetExportEnergy(){
	// Register is cleared after reading
  // response is in "0.1 pulse"
	unsigned short eenergy = transport.Communicate(1, ANenergy, 0xFFFF);
	return (double)eenergy / (PULSE_CONSTANT * 10); // return value is in kWh
}

unsigned short ATM90E26::GetSysStatus(){
	return transport.Communicate(1,SysStatus,0xFFFF);
}










