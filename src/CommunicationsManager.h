#ifndef communicationsmanager_h
#define communicationsmanager_h

#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <SensorManager.h>

#define DEFAILT_MQTT_PORT 1883

class CommunicationsManager {

    public:

        bool subscribeActuator();
        bool publishSensor(String, String);
        void handleMessageMQTT(char*, byte*, unsigned int);
        void loop();

        CommunicationsManager(WiFiClient, String, String, int, 
            std::function<void(char*, byte*, unsigned int)>);
        CommunicationsManager(WiFiClient, String, String, 
            std::function<void(char*, byte*, unsigned int)>);
        CommunicationsManager();

        void setServer(char*, int);
        void setServer(int);
        void setServer(char*);
        void setLocation(String);
        void setMQTTPort(char*);
        void setMQTTCredentials(char*, char*);

        
    private:

        String clientID;
        String location;
        char* serverAddress;
        int serverPort;
        char* userName;
        char* userPassword;
        PubSubClient mqttClient;

        void reconnect();
};

#endif
