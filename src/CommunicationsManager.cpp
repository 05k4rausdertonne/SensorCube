
#include <CommunicationsManager.h>

CommunicationsManager::CommunicationsManager(WiFiClient wifiClient, String id,
    std::function<void(char*, byte*, unsigned int)> messageCallbackMQTT)
{
    
    mqttClient = PubSubClient(wifiClient);

    mqttClient.setCallback(messageCallbackMQTT);

    clientID = id;

    location = "tbd"; //default location, placeholder

}


void CommunicationsManager::setServerAddress(char* sAddr)
{
    serverAddress = sAddr;
    mqttClient.setServer(serverAddress, 1883);
}

void CommunicationsManager::setLocation(String loc)
{
    location = loc;
}


bool CommunicationsManager::publishSensor(String payload, String sensorType)
{
    if (!mqttClient.connected())
    {
        reconnect();
    }

    String outTopic = "sensor/";
    outTopic.concat(sensorType + "/");
    outTopic.concat(clientID + "/");
    outTopic.concat(location);

    if(mqttClient.publish(outTopic.c_str(), payload.c_str()))
    {
        return true;
    }
    else   
    {
        return false;
    }
}

bool CommunicationsManager::subscribeActuator()
{
    return true;
    // TODO needs to be populated once ActuatorManager exists
}

void CommunicationsManager::handleMessageMQTT(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    }
    Serial.println();
}

void CommunicationsManager::loop()
{
    if (!mqttClient.connected())
    {
        reconnect();
    }
    mqttClient.loop();
}

void CommunicationsManager::reconnect()
{
    if (mqttClient.connect((char*)WiFi.macAddress().c_str()))
    {
        // mqttClient.publish("outTopic69","hello world");

        // mqttClient.subscribe("inTopic69");
    }
}
