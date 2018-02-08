/* ATM90E26 Energy Monitor Demo Application

    The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ENERGYIC_H__
#define __ENERGYIC_H__

#include <Arduino.h>
#include "ATM90E26.h"

struct EnergyIC_CalibrationSettings
{
	// PL_Constant
	double referenceVoltage_volts;
	double baseCurrent_amps;
	double currentTransformerRange_amps;
	double currentTransformerOutput_milliamps;
	double currentBurdenResistor_ohms;
	double voltageTransformerOutput_volts;
	double voltageDividerResistor1_ohms;
	double voltageDividerResistor2_ohms;

	// Gains
	double actualMainsVoltage_volts;
	double reportedVoltage_volts;
	double actualMainsCurrent_amps;
	double reportedCurrent_amps;
};

class EnergyIC
{
	public:
		EnergyIC(ATM90E26& atm90e26) : atm90e26(atm90e26) {};
		void Initialize(EnergyIC_CalibrationSettings settings);
		unsigned short GetSysStatus();
		unsigned short  GetMeterStatus();
		double GetLineVoltage();
		double GetLineCurrent();
		double GetActivePower();
		double GetFrequency();
		double GetPowerFactor();
		double GetImportEnergy();
		double GetExportEnergy();
	private:
		ATM90E26& atm90e26;
};

#endif
