#include <Wire.h>

// All values are floats represented as ints: (uint)(real_value * 1000)
volatile uint8_t i2c_regs[] = 
{
    0, // supply voltage low byte
    0, // supply voltage high byte
    0, // ICAL channel 0 low byte
    0, // ICAL channel 0 high byte
    0, // ICAL channel 1 low byte
    0, // ICAL channel 1 high byte
    0, // Channel 0 current low byte 
    0, // Channel 0 current high byte 
    0, // Channel 1 current low byte 
    0  // Channel 1 current high byte
};
#define SUPPLY_VOLTAGE_REGISTER 0
#define ICAL0_REGISTER 2
#define ICAL1_REGISTER 4
#define CHANNEL0_REGISTER 6
#define CHANNEL1_REGISTER 8
const byte reg_size = sizeof(i2c_regs);
// Tracks the current register pointer position
volatile byte reg_position;


#define ADC_BITS    10
#define ADC_COUNTS  (1 << ADC_BITS)
double offsetI = (ADC_COUNTS >> 1);

double supplyVoltage = 0;
double calibration0 = 0;
double calibration1 = 0;

void setup() {
    Wire.begin(8);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
}

void loop()
{
    // We take readings continuously and the master will get
    // the most recent reading
    TakeReading(0, 1676, calibration0, CHANNEL0_REGISTER);
    TakeReading(1, 1676, calibration1, CHANNEL1_REGISTER);
 }

void TakeReading(byte analogPin, unsigned int numberOfSamples, double calibration, unsigned int registerIndex)
{
    double current = calcIrms(analogPin, numberOfSamples, calibration);
    current = abs(current);
    unsigned int registerValue = (unsigned int)(current * 1000.0);
    
    cli();
    i2c_regs[registerIndex] = lowByte(registerValue);
    i2c_regs[registerIndex + 1] = highByte(registerValue);
    sei();
}

double calcIrms(byte analogPin, unsigned int Number_of_Samples, double calibration)
{
    // Borrowed from https://github.com/openenergymonitor/EmonLib

    int sampleI = 0;
    double sumI = 0;

    // Throw away the first read because we're probably switching inputs
    analogRead(analogPin);

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

    double I_RATIO = calibration * (supplyVoltage / (ADC_COUNTS));
    double Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

    return Irms;
}

void UpdateConfiguration()
{
    supplyVoltage = (unsigned int)(i2c_regs[SUPPLY_VOLTAGE_REGISTER + 1] << 8 | i2c_regs[SUPPLY_VOLTAGE_REGISTER]) / 1000.0;
    calibration0 = (unsigned int)(i2c_regs[ICAL0_REGISTER + 1] << 8 | i2c_regs[ICAL0_REGISTER]) / 1000.0;
    calibration1 = (unsigned int)(i2c_regs[ICAL1_REGISTER + 1] << 8 | i2c_regs[ICAL1_REGISTER]) / 1000.0;
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
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
    }

    UpdateConfiguration();
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
