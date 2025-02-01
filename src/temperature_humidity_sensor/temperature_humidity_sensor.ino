
#include "SHT31.h"

#define PIN_BUZZER 15

SHT31 sht;

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    pinMode(PIN_BUZZER, INPUT_PULLDOWN); // prevent random signal noise from triggering the buzzer
}


void loop()
{
    sht.read();

    Serial.print("temperature: ");
    Serial.print(sht.getTemperature(), 2);
    Serial.print(", humidity: ");
    Serial.println(sht.getHumidity(), 2);

    delay(1000);
}

