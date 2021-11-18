#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// http interface libraries
#include <ESP8266WebServer.h>

// handling serialisation and deserialization of json documents
#include <ArduinoJson.h>

// c++ core libraries
#include <map>
#include <vector>
#include <iterator>

// low level interface libraries
#include <SPI.h>
#include <Wire.h>

// library for accessing the mfrc522 ic
#include <MFRC522.h>

// library for accessing the BME680 sensor ic
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// define spi connection pins
#define RST_PIN         D0 
#define SS_PIN          D8 

// TODO C++ ist müll, der scheiß muss in einen file -_-

// init Wifi
WiFiClient wifiClient;

// init web server
ESP8266WebServer  server(80);

// init mqtt client
PubSubClient mqttClient(wifiClient);

// Create MFRC522 instance
MFRC522 mfrc522(SS_PIN, RST_PIN);  

// Create BME680 instance in I2C configuration
Adafruit_BME680 bme; 

// init global args with default parameters
std::map<String, String> globalArgs = {
    {"location", "tbd"},
    {"address", "broker.hivemq.com"},
    {"port", "1883"},
    {"username", "tbd"},
    {"password", "tbd"},
    {"uuid", "tbd"},
    {"sealevelpressure", "1013.25"}
};

//--------------------
//
// sensor utils
//
//--------------------

struct sensor
{
    unsigned long lastScanned;
    unsigned int scanInterval;
    String (*readSensor)(String type);
    String lastValue;
    unsigned long lastNewValue;
    unsigned int bounceTime;
};

// bme680 functions
void initBME680()
{
    if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    }

    // Set up bme680 oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
}

//callback function for collecting readings from bme680
String readBME680(String type)
{   
    // make synchronous reading needs to be changed to async if that takes to long
    // mb make all readings async?
    if (! bme.performReading()) 
    {
        Serial.println("Failed to perform reading :(");
        return "";
    }

    String value; 

    if(type ==  "temperature") value = (String)bme.temperature;
    else if(type ==  "pressure") value = (String)bme.pressure;
    else if(type ==  "humidity") value = (String)bme.humidity;
    else if(type ==  "gas_resistance") value = (String)bme.gas_resistance;
    
    return value;
}

// MFRC522 functions
void initMFRC522()
{
    // init RFID shield
    SPI.begin();			// Init SPI bus
    mfrc522.PCD_Init();		// Init MFRC522

    delay(5);

    Serial.print("RFID tag reader ");
    mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details

}

String readMFRC522(String type)
{

    String value = "";

    if(type == "nfcID")
    {
        if (mfrc522.PICC_IsNewCardPresent())  // (true, if RFID tag/card is present ) PICC = Proximity Integrated Circuit Card
        {

            if(mfrc522.PICC_ReadCardSerial()) { // true, if RFID tag/card was read
                
                for (byte i = 0; i < mfrc522.uid.size; ++i) { // read id (in parts)
                
                value.concat(String(mfrc522.uid.uidByte[i], HEX)); // print id as hex values
                }
            }
        }
    } 

    return value;
}

// factory method for sensor struct
sensor makeSensor(
    unsigned int scanInterval,
    String (*readSensor)(String type),
    unsigned int bounceTime
    )
{
    return {
        0,
        scanInterval,
        readSensor,
        "",
        0,
        bounceTime
    };
}

// vector for storing sensors
std::map<String, sensor> sensors =
{
    {"temperature", makeSensor(10000, readBME680, 0)},
    {"pressure", makeSensor(10000, readBME680, 0)},
    {"humidity", makeSensor(10000, readBME680, 0)},
    {"gas_resistance", makeSensor(10000, readBME680, 0)},
    {"altitude", makeSensor(10000, readBME680, 0)},
    {"nfcID", makeSensor(100, readMFRC522, 5000)}
};

// init function for the sensors
void initSensors()
{
    // call individual sensor inits here
    initBME680();
    initMFRC522();
}



// loop function for the sensors
void loopSensors()
{
    for (std::map<String, sensor>::iterator it = sensors.begin(); it != sensors.end(); it++)
    {
        if(millis() - it->second.lastScanned > it->second.scanInterval)
        {
            String value = it->second.readSensor(it->first);
            
            if(value != it->second.lastValue
                && millis() - it->second.lastNewValue > it->second.bounceTime)
            {
                publishSensor(value, it->first);

                it->second.lastValue = value;
                it->second.lastNewValue = millis();

            }
                
            it->second.lastScanned = millis();
        }
    }
}

// ---------------------------
//
// actuator utils
//
// ---------------------------

// struct actuatorValue
// {
//     String name;
//     int current;
//     int target;
//     +
// }

// struct actuator
// {
//     std::map<String, int>;
//     String (*writeActuator)(String type);
//     String lastValue;
//     unsigned long lastNewValue;
//     unsigned int bounceTime;
// };

// void initActuators()
// {

// }

// void loopActuators()
// {

// }


// ---------------------------
//
// MQTT client helper functions
//
// ---------------------------
void reconnectMQTT()
{   
    Serial.println("attempting reconnect to MQTT broker..");
    if (mqttClient.connect((char*)WiFi.macAddress().c_str()))
    {
        // mqttClient.publish("outTopic69","hello world");
        Serial.println("reconnect successfull :)");

        mqttClient.subscribe("inTopic69");
    }
    else
    {
        Serial.println("reconnect failed :(");
    }
}

void setMQTTServer()
{
    mqttClient.disconnect();

    mqttClient.setServer(globalArgs.find("address")->second.c_str(), 1883);
                
}

void handleMessageMQTT(char* topic, byte* payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    }
    Serial.println();
}

bool publishSensor(String payload, String sensorType)
{

    if(payload.equals(""))
    {
        return false;
    }

    String outTopic = "sensor/";
    outTopic.concat(sensorType + "/");
    outTopic.concat(globalArgs.find("uuid")->second + "/");
    outTopic.concat(globalArgs.find("location")->second);

    if(mqttClient.publish(outTopic.c_str(), payload.c_str()))
    {
        Serial.println(outTopic);
        Serial.println(payload);
        return true;
    }
    else   
    {
        Serial.print("publish failed: ");
        Serial.println(sensorType);
        return false;
    }
}

// ---------------------------
//
// REST interface helper functions
//
// ---------------------------

String serializeConfig()
{
    DynamicJsonDocument doc(JSON_OBJECT_SIZE(globalArgs.size() 
        + JSON_ARRAY_SIZE(sensors.size())
        + sensors.size() * JSON_OBJECT_SIZE(2))
        + JSON_ARRAY_SIZE(1));

    for (std::map<String, String>::iterator it = globalArgs.begin(); it != globalArgs.end(); it++)
    {
        doc[it->first] = it->second;
    }

    doc.createNestedArray("sensors");

    for (std::map<String, sensor>::iterator it = sensors.begin(); it != sensors.end(); it++)
    {
        doc["sensors"][distance(sensors.begin(), it)]["type"] = it->first;
        doc["sensors"][distance(sensors.begin(), it)]["scanInterval"] = it->second.scanInterval;
    }

    doc.createNestedArray("actuators");

    String response;

    serializeJson(doc, response);

    Serial.println(response);

    return response;
}

// request callback for root
void handleRootRequest()
{
    Serial.print("request incomming: ");
    Serial.println(server.arg("plain"));

    if (server.method() != HTTP_POST) 
    {
        server.send (200, "text/json", serializeConfig());
    }
    else 
    {
        StaticJsonDocument<256> doc;
        deserializeJson(doc, server.arg("plain"));
        
        for (std::map<String, String>::iterator it = globalArgs.begin(); it != globalArgs.end(); it++)
        {
            if (doc.containsKey(it->first))
            {
                Serial.print("Arg " + it->first + " set from " + it->second + " to ");
                
                globalArgs.at(it->first) = (String)doc[it->first];

                setMQTTServer();
                
                Serial.println(it->second);                
            }
        }
        server.send(200, "text/plain", serializeConfig());
    }
}

// request callback for sensor config
void handleSensorRequest()
{
    if (server.method() != HTTP_POST) 
    {
        server.send (200, "text/json", serializeConfig());
    }
    else 
    {
        StaticJsonDocument<128> doc;
        deserializeJson(doc, server.arg("plain"));
        
        for (std::map<String, sensor>::iterator it = sensors.begin();
            it != sensors.end(); 
            it++)
        {
            if (doc.containsKey(it->first) 
                && doc[it->first].containsKey("scanInterval"))
            {
                it->second.scanInterval = doc[it->first]["scanInterval"];
            }
        }

        server.send(200, "text/json", serializeConfig());
    }
}

// callback for not found
void handleNotFound()
{
    server.send(404, "text/plain", "not found. go somewhere else.");
}



//-------------------------------------------

void initComms()
{
    //init WiFi connection
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    
    WiFiManager wifiManager;
    wifiManager.autoConnect(WiFi.macAddress().c_str(), "einszweidreivier");
    
    Serial.println();
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    // configure mqtt connection
    mqttClient.setCallback(handleMessageMQTT);
    mqttClient.setServer(
        globalArgs.find("address")->second.c_str(), 
        globalArgs.find("port")->second.toInt());

    server.on("/", handleRootRequest);
    server.on("/sensor", handleSensorRequest);
    server.onNotFound(handleNotFound);

    server.begin();
}

void loopComms()
{
    server.handleClient();
    // handle MQTT client
    if (!mqttClient.connected())
    {
        reconnectMQTT();
        return;
    }
    mqttClient.loop();
}

// ---------------------------
//
// main setup function
//
// ---------------------------
void setup()
{    
    Serial.begin(115200);
    while (!Serial);

    initComms();

    initSensors();

    initActuators();

    delay(50);
}

// ---------------------------
//
// main loop function
//
// ---------------------------
void loop()
{   
    loopComms();

    loopSensors();

    loopActuators();
}



