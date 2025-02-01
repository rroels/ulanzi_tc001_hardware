
#define PIN_BUZZER          15
#define PIN_BATTERY_VOLTAGE 34

void setup() {
    Serial.begin(115200);

    pinMode(PIN_BUZZER, INPUT_PULLDOWN); // prevent random signal noise from triggering the buzzer
    pinMode(PIN_BATTERY_VOLTAGE, INPUT);
}

void loop() {
    int raw = analogRead(PIN_BATTERY_VOLTAGE);
    float voltage = getBatteryVoltage();

    Serial.print("raw: ");
    Serial.println(raw);

    Serial.print("voltage: ");
    Serial.println(voltage);

    delay(1000);
}

// credits: https://github.com/aptonline/PixelIt_Ulanzi/blob/main/src/PixelIt.ino
float getBatteryVoltage()
{
    float batteryLevelPct = map(analogRead(PIN_BATTERY_VOLTAGE), 510, 660, 0, 100);
    if (batteryLevelPct >= 100) { batteryLevelPct = 100; }
    if (batteryLevelPct <= 0) { batteryLevelPct = 1; }
    return batteryLevelPct;
}

