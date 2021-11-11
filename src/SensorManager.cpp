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
                // communicationsManager->publishSensor(
                // value,
                // (*it)->getName());

                Serial.println(value);
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

//TODO fick  polimorhismus in c++ du musst dir was anderes suchen -_-
bool SensorManager::addSensor(Sensor* sensor)
{
    Serial.print("sensor: ");
    Serial.println(sensor->getName());

    Serial.println(sensor->getRate());

    if(getSensorByName(sensor->getName()))
    {
        return false;
    }
    else
    {
        
        sensors.push_back(*sensor);
        Serial.println(sensors.size());

        return true;
    }
}

