/* ATM90E26 Energy Monitor Demo Application

    The MIT License (MIT)

  Copyright (c) 2016 whatnick and Ryzee

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
 
#include <SPI.h>
#include "ATM90E26_SPI.h"

ATM90E26_SPI::ATM90E26_SPI(int pin)
{
    _cs = pin;
}

void ATM90E26_SPI::Initialize()
{
    pinMode(_cs, OUTPUT );

    SPI.begin();

    #if defined(ENERGIA)
        SPI.setBitOrder(MSBFIRST);
        SPI.setDataMode(SPI_MODE3);
        SPI.setClockDivider(SPI_CLOCK_DIV16);
    #endif
}

unsigned short ATM90E26_SPI::Communicate(unsigned char RW, unsigned char address, unsigned short val)
{
    unsigned char* data=(unsigned char*)&val;
    unsigned short output;
    //SPI interface rate is 200 to 160k bps. It Will need to be slowed down for EnergyIC
    SPISettings settings(200000, MSBFIRST, SPI_MODE3);
   
	//switch MSB and LSB of value
	output=(val>>8)|(val<<8);
	val=output;
	
	//Set read write flag
	address|=RW<<7;
	
	//Transmit and receive data
    #if !defined(ENERGIA)
    SPI.beginTransaction(settings);
    #endif
    //Disable LoRa chip on M0-LoRa
    //digitalWrite (8,HIGH);     
    digitalWrite (_cs,LOW);
    delayMicroseconds(10);
    SPI.transfer(address);
    /* Must wait 4 us for data to become valid */
    delayMicroseconds(4);

  //Read data
  //Do for each byte in transfer
  if(RW)
  {  
    for (byte i=0; i<2; i++)
    {
      *data = SPI.transfer (0x00);
      data++;
    }
	//Transfer16 is not valid on Energia
	//val = SPI.transfer16(0x00);
  }
  else
  {
    for (byte i=0; i<2; i++)
    {
      SPI.transfer(*data);             // write all the bytes
      data++;
    }
	//Transfer16 is not valid on Energia
	//SPI.transfer16(val);
  }
  
	digitalWrite(_cs,HIGH);
  //Reenable LoRa chip on M0-LoRa
  //digitalWrite(8,LOW);
  delayMicroseconds(10);
  #if !defined(ENERGIA)
  SPI.endTransaction();
  #endif
        
	output=(val>>8)|(val<<8); //reverse MSB and LSB
	return output;
	//Use with transfer16
	//return val;
}
