#include "RTClib.h"

#define PIN_BUZZER 15

RTC_DS1307 rtc;
DateTime now;
time_t unix_time;

void setup () {
    Serial.begin(115200);

    pinMode(PIN_BUZZER, INPUT_PULLDOWN); // prevent random signal noise from triggering the buzzer

    // set the time to the time of compilation
    if (!rtc.isrunning()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
}

void loop () {
    now = rtc.now();
    unix_time = now.unixtime();

    Serial.print("time : ");
    Serial.print(unix_time);
    Serial.print(", ");
    Serial.print(ctime(&unix_time));

    delay(1000);
}