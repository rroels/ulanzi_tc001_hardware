
#include <LightDependentResistor.h>

#define PIN_BUZZER          15
#define PIN_LIGHT_SENSOR    35

LightDependentResistor photocell(PIN_LIGHT_SENSOR,
                                 10000, // 10k pulldown-resistor
                                 LightDependentResistor::GL5516,
                                 12,    // ESP32 default ADC resolution
                                 0);    // smoothing of 0

void setup() {
    Serial.begin(115200);

    pinMode(PIN_BUZZER, INPUT_PULLDOWN); // prevent random signal noise from triggering the buzzer
    pinMode(PIN_LIGHT_SENSOR, INPUT);

    photocell.setPhotocellPositionOnGround(false);
}

void loop() {

    int raw = analogRead(PIN_LIGHT_SENSOR);
    float lux = photocell.getCurrentLux();

    Serial.print("raw: ");
    Serial.println(raw);

    Serial.print("lux: ");
    Serial.println(lux);

    delay(1000);
}

