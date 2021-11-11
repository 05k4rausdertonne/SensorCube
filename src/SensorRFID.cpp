#include <SensorRFID.h>

#define SAME_ID_TIMER 5000

// TODO: check why cards aren't being scanned

SensorRFID::SensorRFID(String name, int rate, MFRC522* board) : Sensor(name, rate)
{

    mfrc522 = board;
    lastID = "";
    lastIDTime = 0;

}

String SensorRFID::readValue()
{
    Serial.println("reading nfc");
    if (!mfrc522->PICC_IsNewCardPresent() && 
    !millis() - lastScan > (unsigned long)sampleRate) // (false, if RFID tag/card is present ) PICC = Proximity Integrated Circuit Card
    {
        return "";
    }

    lastScan = millis();

    String tagID = "";

    if (!mfrc522->PICC_ReadCardSerial())  // false, if RFID tag/card was read
    {
        return "";    
    }

    for (byte i = 0; i < mfrc522->uid.size; ++i) { // read id (in parts)
      
        tagID.concat(String(mfrc522->uid.uidByte[i], HEX)); // print id as hex values
    }

    if (tagID == lastID && millis() - lastIDTime < SAME_ID_TIMER)
    {
        return "";
    }

    lastIDTime = millis();

    return tagID;
}