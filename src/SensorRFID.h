#ifndef sensor_rfid_h
#define sensor_rfid_h

#include <Arduino.h>
#include <Sensor.h>
#include <MFRC522.h>


class SensorRFID: public Sensor 
{

    public:
        SensorRFID(String, int, MFRC522*);

        String readValue();
        bool needsPolling();
        String getSensorType();
        int getID();
        
    private:
        String sensorName;
        int sampleRate;
        unsigned long lastScan;

        MFRC522* mfrc522;        
        String lastID;
        unsigned long lastIDTime;
        
};

#endif