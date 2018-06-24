#include <Wire.h>

volatile uint8_t i2c_regs[] =
{
    0x01, // Status register, writing (1<<7 | channel) will start a conversion on that channel, the flag will be set low when conversion is done.
    0x02, // first byte of float value
    0x03,
    0x04,
    0x05  // last byte
};
const byte reg_size = sizeof(i2c_regs);
// Tracks the current register pointer position
volatile byte reg_position;
// Tracks whether to do a conversion cycle
volatile boolean do_conversion;

#define ADC_BITS    10
#define ADC_COUNTS  (1 << ADC_BITS)
#define ICAL 100
double offsetI = (ADC_COUNTS >> 1);
int SupplyVoltage = 3300;

void setup() {
    Wire.begin(8);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
}

void loop()
{
    if (do_conversion)
    {
        byte analogPin = i2c_regs[0] & 0x7f;

        digitalWrite(8, HIGH);
        float value = calcIrms(analogPin, 1000);
        digitalWrite(8, LOW);

        byte* valueAsBytes = (byte*)&value;

        cli();
        i2c_regs[1] = *valueAsBytes;
        i2c_regs[2] = *(valueAsBytes + 1);
        i2c_regs[3] = *(valueAsBytes + 2);
        i2c_regs[4] = *(valueAsBytes + 3);
        sei();

        // And clear the conversion flag so the master knows we're ready
        bitClear(i2c_regs[0], 7);
        
        do_conversion = false;
    }
 }

double calcIrms(byte analogPin, unsigned int Number_of_Samples)
{
    // Borrowed from https://github.com/openenergymonitor/EmonLib

    int sampleI = 0;
    double sumI = 0;
    
    for (unsigned int n = 0; n < Number_of_Samples; n++)
    {
        sampleI = analogRead(analogPin);

        // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
        //  then subtract this - signal is now centered on 0 counts.
        offsetI = (offsetI + (sampleI - offsetI) / 1024);
        double filteredI = sampleI - offsetI;

        // Root-mean-square method current
        // 1) square current values
        double sqI = filteredI * filteredI;
        // 2) sum
        sumI += sqI;
    }

    double I_RATIO = ICAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
    double Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

    return Irms;
}

void receiveEvent(int howMany)
{
    if (howMany < 1 || howMany > BUFFER_LENGTH)
    {
        return;
    }

    reg_position = Wire.read();
    howMany--;
    if (howMany == 0)
    {
        // This write was only to set the buffer for next read
        return;
    }
    
    while (howMany--)
    {
        i2c_regs[reg_position] = Wire.read();
        if (reg_position == 0 // If it was the first register
            && bitRead(i2c_regs[0], 7) // And the highest bit is set
            && !do_conversion // and we do not actually have a conversion running already
            )
        {
            do_conversion = true;
        }
        
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
    }
}

void requestEvent()
{
    // The Wire library calls this event once per request, not once per byte as it
    // probably ought to. The workaround is to just dump the entire register set
    // into the send buffer and the master can clock back as many bytes as it wants.
    // When the master issues a stop then the rest of our send buffer will be discarded.
    
    for (int x = 0; x < reg_size; x++)
    {
        Wire.write(i2c_regs[reg_position]);
    
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
    }
}
