
#define PIN_BUZZER 15

uint8_t freq_index = 0;
uint8_t freq_count = 4;
int freqs[4] = {247, 262, 294, 330};

void setup() {
    Serial.begin(115200);
    pinMode(PIN_BUZZER, INPUT_PULLDOWN);
}

void loop() {
    tone(PIN_BUZZER, freqs[freq_index], 500);
    delay(500);

    if(++freq_index == freq_count){freq_index = 0;}
}
