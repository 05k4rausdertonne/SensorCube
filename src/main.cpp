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
// define spi connection pins for the mfrc522 ic
#define RST_PIN         D0 
#define SS_PIN          D8 

// library for accessing the BME680 sensor ic
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// Library for controlling the ws2812b leds
#include <FastLED.h>
// define pin for led connection
#define LED_PIN         D4
#define NUM_LEDS        1

// init led array
CRGB leds [NUM_LEDS];

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

// init global loop timer and time delta
unsigned long lastLoop = 0;
unsigned int lastDelta = 1;

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

    if (type ==  "temperature") value = (String)bme.temperature;
    else if (type ==  "pressure") value = (String)bme.pressure;
    else if (type ==  "humidity") value = (String)bme.humidity;
    else if (type ==  "gas_resistance") value = (String)bme.gas_resistance;
    
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

            if (mfrc522.PICC_ReadCardSerial()) { // true, if RFID tag/card was read
                
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
    {"nfc_id", makeSensor(100, readMFRC522, 5000)}
};



// ---------------------------
//
// actuator utils
//
// ---------------------------

struct actuatorValue
{
    int current;
    int target;
    unsigned int timeToTarget;    
};

struct actuator
{
    std::map<String, actuatorValue> values;
    void (*writeActuator)(std::map<String, actuator>::iterator actuatorRef);
};

// factory functions for actuators
actuator makeActuator(
    String valueNames[], 
    void (*writeActuator)(std::map<String, actuator>::iterator actuatorRef))
{
    std::map<String, actuatorValue> newValues = {};

    for (int i = 0; i < (sizeof(valueNames)-1); i++)
    {
        newValues.insert_or_assign(
            valueNames[i],
            (actuatorValue) {0, 0, 0}
        );
        Serial.println(i);
    }

    return {
        newValues,
        writeActuator
    };
}

void writeLED(std::map<String, actuator>::iterator actuatorRef)
{
    unsigned int ledIndex = actuatorRef->first.substring(7).toInt();

    leds[ledIndex] = CHSV(actuatorRef->second.values.find("hue")->second.current,
        actuatorRef->second.values.find("sat")->second.current,
        actuatorRef->second.values.find("val")->second.current);

    Serial.print(actuatorRef->second.values.find("hue")->second.current);
    Serial.print(" ");
    Serial.print(actuatorRef->second.values.find("sat")->second.current);
    Serial.print(" ");
    Serial.println(actuatorRef->second.values.find("val")->second.current);

    FastLED.show();
}

std::map<String, actuator> actuators =
{
    // {"led000", makeActuator({"hue", "sat", "val"}, writeLED)}
};

void initLED()
{
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    
    for (int i = 0; i<NUM_LEDS; i++)
    {
        leds[i] = CHSV(0, 0, 0);

        String ledName = "led";
        ledName.concat((String)(i % 1000));
        ledName.concat((String)(i % 100));
        ledName.concat((String)(i % 10));        

        String valueNames[] = {"hue", "sat", "val"};
        actuators.insert_or_assign(
            ledName,
            makeActuator(valueNames, writeLED));

        Serial.println("added " + ledName);

        FastLED.show();
    }
}

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

        String subTopic = "actuator/+/";
        subTopic.concat(globalArgs.find("location")->second);
        subTopic.concat("/#");

        mqttClient.subscribe(subTopic.c_str());

        Serial.println("subscribed to topic: "+ subTopic);
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

    String topicString = topic;
    for (std::map<String, actuator>::iterator it = actuators.begin(); 
        it != actuators.end(); 
        it++)
    {
        if (topicString.startsWith("actuator/" + it->first))
        {
            for (std::map<String, actuatorValue>::iterator val = it->second.values.begin(); 
                val != it->second.values.end(); 
                val++)
            {
                if (topicString.endsWith(val->first))
                {
                    Serial.print("setting " + val->first + " at " + it->first);
                
                    StaticJsonDocument<64> doc;
                    deserializeJson(doc, (char*)payload, length);

                    if (doc.containsKey("value"))
                    {
                        Serial.print(" to " + (String)doc["value"]);
                        val->second.target = doc["value"];

                        if (doc.containsKey("time"))
                        {
                            Serial.print(" over " + (String)doc["time"] + " milliseconds");
                            val->second.timeToTarget = doc["time"];
                        }
                    }

                    Serial.println(".");
                }
            }
        }
    }

    
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

    for (std::map<String, String>::iterator it = globalArgs.begin(); 
        it != globalArgs.end(); 
        it++)
    {
        doc[it->first] = it->second;
    }

    doc.createNestedArray("sensors");

    for (std::map<String, sensor>::iterator it = sensors.begin(); 
        it != sensors.end(); 
        it++)
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

// request callback for actuator values
void handleActuatorRequest()
{
    if (server.method() != HTTP_POST) 
    {
        server.send (200, "text/json", serializeConfig());
    }
    else 
    {        
        server.send(200, "text/json", serializeConfig());
    }
}

// callback for not found
void handleNotFound()
{
    server.send(404, "text/plain", "not found. go somewhere else.");
}



// ---------------------------
//
// init and loop functions
//
// ---------------------------

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
        if (millis() - it->second.lastScanned > it->second.scanInterval)
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

void initActuators()
{
    initLED();
}

void loopActuators()
{
    for (std::map<String, actuator>::iterator actuatorRef = actuators.begin(); 
        actuatorRef != actuators.end(); 
        actuatorRef++)
    {
        for (std::map<String, actuatorValue>::iterator valueRef = actuatorRef->second.values.begin();
            valueRef != actuatorRef->second.values.end();
            valueRef++)
        {
            if (valueRef->second.current != valueRef->second.target)
            {
                // Serial.println((String)valueRef->second.current + " " + (String)valueRef->second.target);
                if (valueRef->second.timeToTarget > lastDelta)
                {
                    valueRef->second.current += 
                        (valueRef->second.target - valueRef->second.current) / 
                            (valueRef->second.timeToTarget / lastDelta);
                }

                else
                {
                    // Serial.println((String)valueRef->second.current);
                    valueRef->second.current = valueRef->second.target;
                }

                actuatorRef->second.writeActuator(actuatorRef);
            }

        }
    }
}

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

    writeLED(actuators.find("led000"));

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
    
    // TODO delta is way to short. find workaround!!
    lastDelta = millis() - lastLoop;
    lastLoop = millis();


}



