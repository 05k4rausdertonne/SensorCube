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


// TODO this is the problem. sensor needs to be abstract
// String Sensor::readValue()
// {
//     // Serial.println("error: abstract function is being called");
//     return "";
// }

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
