/* ATM90E26 Energy Monitor Demo Application

    The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __ATM90E26_SPI_H__
#define __ATM90E26_SPI_H__

#include "ATM90E26_Transport.h"

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
//const int energy_IRQ = 2;
#ifdef  ARDUINO_ESP8266_WEMOS_D1MINI  // WeMos mini and D1 R2
const int energy_CS = D8; // WEMOS SS pin
#endif

#ifdef  ARDUINO_ESP8266_ESP12  // Adafruit Huzzah
const int energy_CS = 15; // HUZZAH SS pins ( 0 or 15)
#endif

#ifdef ARDUINO_ARCH_SAMD //M0 board
const int energy_CS = 10; // M0 SS pin
#endif 

#ifdef __AVR_ATmega32U4__ //32u4 board
const int energy_CS = 10; // 32u4 SS pin
#endif 

#if !(defined ARDUINO_ESP8266_WEMOS_D1MINI || defined ARDUINO_ESP8266_ESP12 || defined ARDUINO_ARCH_SAMD || defined __AVR_ATmega32U4__)
const int energy_CS = SS; // Use default SS pin for unknown Arduino
#endif

class ATM90E26_SPI : public ATM90E26_Transport
{
	public:
		ATM90E26_SPI(int pin = energy_CS);
		void Initialize();
		unsigned short Communicate(unsigned char RW, unsigned char address, unsigned short val);
	private:
		int _cs;
};

#endif
