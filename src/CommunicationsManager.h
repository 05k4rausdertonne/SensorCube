#ifndef communicationsmanager_h
#define communicationsmanager_h

#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <SensorManager.h>

class CommunicationsManager {

    public:

        bool subscribeActuator();
        bool publishSensor(String payload, String sensorName);
        void handleMessageMQTT(char*, byte*, unsigned int);
        void loop();

        CommunicationsManager(WiFiClient, String, std::function<void(char*, byte*, unsigned int)>);
        CommunicationsManager();

        void setServerAddress(char* sAddr);
        void setLocation(String loc);
        void setMQTTPort(char* mqttPort);

        
    private:

        String clientID;
        String location;
        char* serverAddress;
        PubSubClient mqttClient;

        void reconnect();
};

#endif
