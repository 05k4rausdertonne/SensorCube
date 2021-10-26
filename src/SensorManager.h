#ifndef sensormanager_h
#define sensormanager_h

#include <Arduino.h>
#include <Sensor.h>
#include <CommunicationsManager.h>
#include <list>
#include <algorithm>
#include <MFRC522.h>



class CommunicationsManager;
class Sensor;
class SensorRFID;

class SensorManager {

    public:

        SensorManager(CommunicationsManager*);
        SensorManager();

        bool addSensor(Sensor);
        Sensor* getSensorByName(String);
        void publishSensors();
        void loop(); //handles communication and interval
        
    private:
        
        std::list<Sensor> sensors;
        CommunicationsManager* communicationsManager;
};

#endif
