#pragma once
#ifndef sensor_h
#define sensor_h

#include <Arduino.h>


class Sensor {

    public:
        virtual String readValue() = 0;
        Sensor(String, int);
        Sensor();
        String getName();
        int getRate();
        int setRate(int);
        unsigned long getLast();
        unsigned long setLast(unsigned long);
        void update();
        
    private:
        String sensorName;
        int sampleRate;
        unsigned long lastScanned;
};

#endif
