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
#include "bsec.h"

// Library for controlling the ws2812b leds
#include <NeoPixelBus.h>

// define pin for led connection
#define LED_PIN         D4
#define NUM_LEDS        1

// init led array
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> leds(NUM_LEDS, LED_PIN);

// init Wifi
WiFiClient wifiClient;

// init web server
ESP8266WebServer  server(80);

// init mqtt client
PubSubClient mqttClient(wifiClient);

// Create MFRC522 instance
MFRC522 mfrc522(SS_PIN, RST_PIN);  

// Create BME680 instance in I2C configuration
Bsec iaqSensor;
String lastBM680Measurement;

// init global args with default parameters
std::map<String, String> globalArgs = {
    {"location", "tbd"},
    {"address", "broker.hivemq.com"}, //192.168.178.28
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

// Helper function definitions

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

void checkIaqSensorStatus(void)
{
    String output;
    if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
        output = "BSEC error code : " + String(iaqSensor.status);
        Serial.println(output);
        for (;;)
            errLeds(); /* Halt in case of failure */
        } else {
        output = "BSEC warning code : " + String(iaqSensor.status);
        Serial.println(output);
        }
    }

    if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
        output = "BME680 error code : " + String(iaqSensor.bme680Status);
        Serial.println(output);
        for (;;)
            errLeds(); /* Halt in case of failure */
        } else {
        output = "BME680 warning code : " + String(iaqSensor.bme680Status);
        Serial.println(output);
        }
    }
}

// bsec functions
void initBSEC()
{
    Wire.begin();

    iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    String output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();

    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();
}

String readBSEC(String type)
{
   
    String output = ""; 

    if (iaqSensor.run()) 
    {
        StaticJsonDocument<256> doc;

        doc["temperature"] = (String)iaqSensor.temperature;
        doc["pressure"] = (String)iaqSensor.pressure;
        doc["humidity"] = (String)iaqSensor.humidity;
        doc["gas_resistance"] = (String)iaqSensor.gasResistance;
        doc["co2"] = (String)iaqSensor.co2Equivalent;
        doc["iaq"] = (String)iaqSensor.staticIaq;
        doc["breath_voc"] = (String)iaqSensor.breathVocEquivalent;

        serializeJson(doc, output);

    } else {
        checkIaqSensorStatus();
        // Serial.println("Failed to perform reading :(");
        return "";
    }

    lastBM680Measurement = output;

    return "";

}


// bme680 functions
void initBME680()
{
    
}

//callback function for collecting readings from bme680
String readBME680(String type)
{       
    String output = lastBM680Measurement; 

    lastBM680Measurement = "";

    return output;
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

    if(type == "nfc")
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

    if (value == "")
    {
        return "";
    }

    StaticJsonDocument<128> doc;

    doc["nfc_id"] = value;

    value = "";

    serializeJson(doc, value);

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
    {"bsec", makeSensor(100, readBSEC, 0)},
    {"bme680", makeSensor(10000, readBME680, 0)},
    {"nfc", makeSensor(100, readMFRC522, 2000)}
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
    void (*writeActuator)(String actuatorName);
};

// factory functions for actuators
actuator makeActuator(
    String valueNames[], 
    void (*writeActuator)(String actuatorName))
{
    std::map<String, actuatorValue> newValues = {};

    for (int i = 0; i < (sizeof(valueNames)-1); i++)
    {
        newValues.insert_or_assign(
            valueNames[i],
            (actuatorValue) {0, 0, 0}
        );
    }

    return {
        newValues,
        writeActuator
    };
}

std::map<String, actuator> actuators = {};

void writeLED(String ledName)
{
    unsigned int ledIndex = ledName.substring(7).toInt();

    leds.SetPixelColor(ledIndex, HsbColor(
        (float)actuators.find(ledName)->second.values.find("hue")->second.current / 255.0,
        (float)actuators.find(ledName)->second.values.find("sat")->second.current / 255.0,
        (float)actuators.find(ledName)->second.values.find("val")->second.current / 255.0
        ));
}

void initLED()
{
    leds.Begin();
    
    for (int i = 0; i<NUM_LEDS; i++)
    {
        leds.SetPixelColor(i, RgbColor(0,0,0));

        String ledName = "led";
        ledName.concat((String)(i % 1000));
        ledName.concat((String)(i % 100));
        ledName.concat((String)(i % 10));        

        String valueNames[] = {"hue", "sat", "val"};
        actuators.insert_or_assign(
            ledName,
            makeActuator(valueNames, writeLED));

        Serial.println("added " + ledName);

        writeLED(ledName);
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
        subTopic.concat(globalArgs.find("uuid")->second);
        subTopic.concat("/");
        subTopic.concat(globalArgs.find("location")->second);
        subTopic.concat("/#");

        mqttClient.subscribe(subTopic.c_str());

        Serial.println("subscribed to topic: "+ subTopic);
    }
    else
    {
        Serial.print("reconnect failed: ");
        Serial.println(mqttClient.state());
    }
}

void setMQTTServer()
{
    mqttClient.disconnect();

    Serial.print(globalArgs.find("address")->second.c_str());
    Serial.print(":");
    Serial.println((int)globalArgs.find("port")->second.c_str());

    mqttClient.setServer(globalArgs.find("address")->second.c_str(), 
        globalArgs.find("port")->second.toInt());
                
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
        if (topicString.startsWith("actuator/" + it->first + "/"))
        {
            
                
            StaticJsonDocument<512> doc;
            deserializeJson(doc, (char*)payload, length);

            for (std::map<String, actuatorValue>::iterator val = it->second.values.begin(); 
                val != it->second.values.end(); 
                val++)
            {
                if (doc.containsKey("value") && doc["value"].containsKey(val->first))
                {
                    Serial.print("setting " + val->first + " at " + it->first);

                    Serial.print(" to " + (String)doc["value"][val->first]);
                    val->second.target = (int)doc["value"][val->first];

                    if (doc.containsKey("time"))
                    {
                        Serial.print(" over " + (String)doc["time"] + " milliseconds");
                        val->second.timeToTarget = (int)doc["time"];
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
        +JSON_ARRAY_SIZE(actuators.size())
        + actuators.size() * (JSON_ARRAY_SIZE(4) + JSON_OBJECT_SIZE(7)));

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
        if(doc["sensors"][distance(sensors.begin(), it)]["type"] != "bsec")
        {
            doc["sensors"][distance(sensors.begin(), it)]["type"] = it->first;
            doc["sensors"][distance(sensors.begin(), it)]["scanInterval"] = it->second.scanInterval;
        }
    }

    doc.createNestedArray("actuators");

    for (std::map<String, actuator>::iterator actuatorIt = actuators.begin(); 
        actuatorIt != actuators.end(); 
        actuatorIt++)
    {
        JsonObject actuatorObj = doc["actuators"].createNestedObject();

        actuatorObj["type"] = actuatorIt->first;
        actuatorObj.createNestedArray("values");

        for (std::map<String, actuatorValue>::iterator valueIt = 
        actuatorIt->second.values.begin(); 
        valueIt != 
        actuatorIt->second.values.end(); 
        valueIt++)
        {
            actuatorObj["values"][distance(actuatorIt->second.values.begin(), valueIt)] = valueIt->first;
        }
    }

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
    initBSEC();
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

                actuatorRef->second.writeActuator(actuatorRef->first);
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
    // mqttClient.setServer(
    //     globalArgs.find("address")->second.c_str(), 
    //     globalArgs.find("port")->second.toInt());

    setMQTTServer();

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
    
    // TODO delta is way to short. find workaround!!
    lastDelta = millis() - lastLoop;
    lastLoop = millis();

    
    leds.Show();

    delay(2);
}



