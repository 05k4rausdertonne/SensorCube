#include <SensorManager.h>

SensorManager::SensorManager(CommunicationsManager* commMan)
{
    sensors = {};
    communicationsManager = commMan;
}

void SensorManager::loop()
{   
    for (std::list<Sensor>::iterator it = sensors.begin(); 
        it != sensors.end();
        ++it)
    {
        if (millis() - it->getLast() > it->getRate())
        {

        String value = it->readValue(); // TODO needs some refinement once concrete sensors are implemented

            if(!(value == ""))
            {
                it->setLast(millis());
                communicationsManager->publishSensor(
                value,
                it->getName());
            }
        } 
    }  
}

Sensor* SensorManager::getSensorByName(String name)
{
    for (std::list<Sensor>::iterator it = sensors.begin(); 
        it != sensors.end(); 
        ++it)
    {
        if (it->getName() == name)
        {
            return &(*it);
        }
    }
    return nullptr;
}

bool SensorManager::addSensor(Sensor sensor)
{
    Serial.println("sensor:");
    Serial.println(sensor.getName());

    if(!getSensorByName(sensor.getName()))
    {
        sensors.push_back(sensor);
        Serial.println(sensors.size());

        return true;
    }
    else
    {
        return false;
    }
}

       