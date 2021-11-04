#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

// http interface libraries
#include <ESP8266WebServer.h>

// low level interface libraries
#include <SPI.h>
#include <Wire.h>

// library for accessing the mfrc522 ic
#include <MFRC522.h>

// library for accessing the BME680 sensor ic
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// c++ core libraries
#include <vector>
#include <map>

// library for the conversion of json documents
#include <ArduinoJson.h>

// include own classes
#include <SensorManager.h>
#include <CommunicationsManager.h>
#include <SensorRFID.h>

// define connection pins
#define RST_PIN         D0 
#define SS_PIN          D8 

// init Wifi
WiFiClient wifiClient;

// init PubSubClient
PubSubClient pubSubClient(wifiClient);

// init web server
ESP8266WebServer  server(80);

// init sensor objects etc.
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
Adafruit_BME680 bme; // I2C configuration
unsigned long bmeScan = 0;

PubSubClient mqttClient(wifiClient);
char* serverAddressMQTT = "broker.hivemq.com";

// init global args with default parameters
// TODO: add mqtt credentials
std::map<String, String> globalArgs = {
    {"location", "tbd"},
    {"address", "broker.hivemq.com"},
    {"uuid", "tbd"},
    {"sealevelpressure", "1013.25"}
};

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


CommunicationsManager communicationsManager(&pubSubClient, 
        (char*)WiFi.macAddress().c_str(), 
        "broker.hivemq.com", 
        handleMessageMQTT);

SensorManager sensorManager(&communicationsManager);

// struct sensorChip
// {
//     bool async;
//     unsigned long scanEnd = 0;
//     bool scanning = false;
//     uint32_t (*startAsyncReading)();
// };

// sensorChip makeSensorChip(
//     bool async,
//     uint32_t (*startAsyncReading)()
//     )
// {
//     if (async)
//     {
//         return {
//         async,
//         0,
//         false,
//         startAsyncReading,
//         };
//     } 
//     else return {false};
// }

// sensorChip makeSensorChip(
//     bool async
// )    
// {
//     return {false};
// }

// struct sensor
// {
//     unsigned long lastScanned;
//     unsigned int scanInterval;
//     sensorChip* chip;
//     String (*readSensor)(String type);
// };

// sensor makeSensor(
//     unsigned int scanInterval,
//     sensorChip* chip,
//     String (*readSensor)(String type)
//     )
// {
//     return {
//         0,
//         scanInterval,
//         chip,
//         readSensor
//     };
// }

// // callback function for reading RFID tags
// String readMFRC522(String type)
// {
//     String value = "";

//     if (mfrc522.PICC_IsNewCardPresent())  // (true, if RFID tag/card is present ) PICC = Proximity Integrated Circuit Card
//     {
//         StaticJsonDocument<48> doc;

//         if(mfrc522.PICC_ReadCardSerial()) { // true, if RFID tag/card was read
            
//             for (byte i = 0; i < mfrc522.uid.size; ++i) { // read id (in parts)
            
//             value.concat(String(mfrc522.uid.uidByte[i], HEX)); // print id as hex values
//             }
//         }
//     }
//     return value;
// }

// //callback function for collecting readings from bme680
// String readBME680(String type)
// {   
//     String value; 

//     if(type ==  "temperature") value = (String)bme.temperature;
//     else if(type ==  "pressure") value = (String)bme.pressure;
//     else if(type ==  "humidity") value = (String)bme.humidity;
//     else if(type ==  "gas_resistance") value = (String)bme.gas_resistance;
//     else if(type ==  "altitude") value = (String)bme.readAltitude(globalArgs.at("sealevelpressure").toFloat());
    
//     return value;
// }

// uint32_t startBME680()
// {
//     return bme.beginReading();
// }

// // MQTT client helper functions
// void reconnectMQTT()
// {   
//     Serial.println("attempting reconnect to MQTT broker..");
//     if (mqttClient.connect((char*)WiFi.macAddress().c_str()))
//     {
//         // mqttClient.publish("outTopic69","hello world");
//         Serial.println("reconnect successfull :)");

//         mqttClient.subscribe("inTopic69");
//     }
//     else
//     {
//         Serial.println("reconnect failed :(");
//     }
// }


// bool publishSensor(String payload, String sensorType)
// {

//     if(payload.equals(""))
//     {
//         return false;
//     }

//     if (!mqttClient.connected())
//     {
//         reconnectMQTT();
//     }

    

//     String outTopic = "sensor/";
//     outTopic.concat(sensorType + "/");
//     outTopic.concat(globalArgs.find("uuid")->second + "/");
//     outTopic.concat(globalArgs.find("location")->second);

//     if(mqttClient.publish(outTopic.c_str(), payload.c_str()))
//     {
//         Serial.println(outTopic);
//         Serial.println(payload);
//         return true;
//     }
//     else   
//     {
//         Serial.println("publish failed");
//         return false;
//     }
// }


// std::map<String, sensorChip> sensorChips =
// {
//     {"bme680", makeSensorChip(true, startBME680)},
//     {"mfrc522", makeSensorChip(false)}
// };

// // vector for storing sensors
// std::map<String, sensor> sensors =
// {
//     {"temperature", makeSensor(10000,&(sensorChips.at("bme680")), readBME680)},
//     {"pressure", makeSensor(10000,&(sensorChips.at("bme680")), readBME680)},
//     {"humidity", makeSensor(10000,&(sensorChips.at("bme680")), readBME680)},
//     {"gas_resistance", makeSensor(10000,&(sensorChips.at("bme680")), readBME680)},
//     {"altitude", makeSensor(10000,&(sensorChips.at("bme680")), readBME680)},
//     {"nfcID", makeSensor(500,&(sensorChips.at("mfrc522")), readMFRC522)}
// };

// String serializeConfig()
// {
//     DynamicJsonDocument doc(JSON_OBJECT_SIZE(globalArgs.size() 
//         + JSON_ARRAY_SIZE(sensors.size())
//         + sensors.size() * JSON_OBJECT_SIZE(2))
//         + JSON_ARRAY_SIZE(1));

//     for (std::map<String, String>::iterator it = globalArgs.begin(); it != globalArgs.end(); it++)
//     {
//         doc[it->first] = it->second;
//     }

//     doc.createNestedArray("sensors");

//     for (std::map<String, sensor>::iterator it = sensors.begin(); it != sensors.end(); it++)
//     {
//         doc["sensors"][distance(sensors.begin(), it)]["type"] = it->first;
//         doc["sensors"][distance(sensors.begin(), it)]["scanInterval"] = it->second.scanInterval;
//     }

//     doc.createNestedArray("actuators");

//     String response;

//     serializeJson(doc, response);

//     Serial.println(response);

//     return response;
// }

// // request callback for root
// void handleRootRequest()
// {
//     Serial.print("request incomming: ");
//     Serial.println(server.arg("plain"));

//     if (server.method() != HTTP_POST) 
//     {
//         server.send (200, "text/json", serializeConfig());
//     }
//     else 
//     {
//         StaticJsonDocument<256> doc;
//         deserializeJson(doc, server.arg("plain"));
        
//         for (std::map<String, String>::iterator it = globalArgs.begin(); it != globalArgs.end(); it++)
//         {
//             if (doc.containsKey(it->first))
//             {
//                 Serial.print("Arg " + it->first + " set from " + it->second + " to ");
                
//                 globalArgs.at(it->first) = (String)doc[it->first];

//                 mqttClient.disconnect();

//                 mqttClient.setServer(globalArgs.find("address")->second.c_str(), 1883);
                
//                 // reconnectMQTT();
                
//                 Serial.println(it->second);                
//             }
//         }
//         server.send(200, "text/plain", serializeConfig());
//     }
// }

// // request callback for sensor config
// void handleSensorRequest()
// {
//     if (server.method() != HTTP_POST) 
//     {
//         server.send (200, "text/json", serializeConfig());
//     }
//     else 
//     {
//         StaticJsonDocument<128> doc;
//         deserializeJson(doc, server.arg("plain"));
        
//         for (std::map<String, sensor>::iterator it = sensors.begin(); it != sensors.end(); it++)
//         {
//             if (doc.containsKey(it->first) 
//                 && doc[it->first].containsKey("scanInterval"))
//             {
//                 it->second.scanInterval = doc[it->first]["scanInterval"];
//             }
//         }

//         server.send(200, "text/json", serializeConfig());
//     }
// }

// callback for not found
void handleNotFound()
{
    server.send(404, "text/plain", "not found. go somewhere else.");
}

// ---------------------------
//
// main setup function
//
// ---------------------------
void setup()
{

    // globalArgs.emplace("location", "tbd");
    
    Serial.begin(115200);
    while (!Serial);

    if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    }

    // init WiFi connection
    WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    
    WiFiManager wifiManager;
    wifiManager.autoConnect(WiFi.macAddress().c_str(), "einszweidreivier");
    
    Serial.println();
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    // init RFID shield
    SPI.begin();			// Init SPI bus
    mfrc522.PCD_Init();		// Init MFRC522

    delay(5);

    Serial.print("RFID tag reader ");
    mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details

    sensorManager.addSensor(SensorRFID("nfcID", 500, &mfrc522));

    delay(50);
}

// ---------------------------
//
// main loop function
//
// ---------------------------
void loop()
{
    sensorManager.loop();
    // TODO see why exception gets thrown here
    // communicationsManager.loop();
}