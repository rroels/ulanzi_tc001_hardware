
#define PIN_BUZZER          15
#define PIN_BUTTON_LEFT 	26
#define PIN_BUTTON_MIDDLE 	27
#define PIN_BUTTON_RIGHT 	14

void setup() {
    Serial.begin(115200);

    pinMode(PIN_BUZZER, INPUT_PULLDOWN); // prevent random signal noise from triggering the buzzer

    pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);
    pinMode(PIN_BUTTON_MIDDLE, INPUT_PULLUP);
    pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
}

void loop() {
    if(digitalRead(PIN_BUTTON_LEFT) == LOW) {
    	Serial.println("left button");
    }

    if(digitalRead(PIN_BUTTON_MIDDLE) == LOW) {
    	Serial.println("middle button");
    }

    if(digitalRead(PIN_BUTTON_RIGHT) == LOW) {
    	Serial.println("right button");
    }

    delay(500);
}

