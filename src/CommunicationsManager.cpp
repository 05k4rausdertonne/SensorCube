
#include <CommunicationsManager.h>

CommunicationsManager::CommunicationsManager(PubSubClient* client, char* id, 
    String address, int port,
    std::function<void(char*, byte*, unsigned int)> messageCallbackMQTT)
{
    
    mqttClient = client;

    mqttClient->setCallback(messageCallbackMQTT);

    clientID = id;

    location = "tbd"; //default location, placeholder

}

CommunicationsManager::CommunicationsManager(PubSubClient* client, char* id, String address,
    std::function<void(char*, byte*, unsigned int)> messageCallbackMQTT)
{
    CommunicationsManager(client, id, address, DEFAILT_MQTT_PORT, messageCallbackMQTT);
}

void CommunicationsManager::setServer(char* addr, int port)
{
    serverAddress = addr;
    serverPort = port;
    mqttClient->setServer(addr, port);
    Serial.println("new address: " + (String)addr + ":" + port);
}

void CommunicationsManager::setServer(int port)
{
    setServer(serverAddress, port);
}

void CommunicationsManager::setServer(char* addr)
{
    setServer(addr, serverPort);
}

void CommunicationsManager::setLocation(String loc)
{
    location = loc;
}


bool CommunicationsManager::publishSensor(String payload, String sensorType)
{
    if (!mqttClient->connected())
    {
        reconnect();
    }

    String outTopic = "sensor/";
    outTopic.concat(sensorType + "/");
    outTopic.concat((String)clientID + "/");
    outTopic.concat(location);

    if(mqttClient->publish(outTopic.c_str(), payload.c_str()))
    {
        Serial.println(outTopic);
        Serial.println(payload);
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
    Serial.println("looping communications");
    if (!mqttClient->connected())
    {
        reconnect();
    }
    mqttClient->loop();
}

void CommunicationsManager::reconnect()
{
    if (mqttClient->connect(clientID))
    {
        // mqttClient.publish("outTopic69","hello world");

        // mqttClient.subscribe("inTopic69");
    }
}
