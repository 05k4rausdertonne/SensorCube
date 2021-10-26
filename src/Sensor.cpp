#include <Sensor.h>


Sensor::Sensor(String name, int rate)
{
    sensorName = name;
    sampleRate = rate;
    lastScanned = 0;
}

Sensor::Sensor()
{
    
}

String Sensor::readValue()
{
    return "";
}

String Sensor::getName()
{
    return sensorName;
}

int Sensor::getRate()
{
    return sampleRate;
}

int Sensor::setRate(int newRate)
{
    return sampleRate = newRate;
}

unsigned long Sensor::getLast()
{
    return lastScanned;
}

unsigned long Sensor::setLast(unsigned long last)
{
    return lastScanned = last;
}
