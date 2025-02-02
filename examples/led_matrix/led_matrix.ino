
#include <Adafruit_GFX.h>
#include <FastLED.h>
#include <FastLED_NeoMatrix.h>

#define PIN_BUZZER          15
#define PIN_LED_MATRIX      32


CRGB matrixleds[256];

FastLED_NeoMatrix *matrix = new FastLED_NeoMatrix(matrixleds, 32, 8, NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG );

const uint16_t colors[] = { matrix->Color(255, 0, 0), matrix->Color(0, 255, 0), matrix->Color(0, 0, 255) };

void setup() {

    pinMode(PIN_BUZZER, INPUT_PULLDOWN); // prevent random signal noise from triggering the buzzer

    FastLED.addLeds<NEOPIXEL,PIN_LED_MATRIX>(matrixleds, 256);
    matrix->begin();
    matrix->setTextWrap(false);
    matrix->setBrightness(40);
    matrix->setTextColor(colors[0]);
}

int x    = 32;
int pass = 0;

void loop() {
    matrix->fillScreen(0);
    matrix->setCursor(x, 0);
    matrix->print(F("Howdy"));
    if(--x < -36) {
        x = matrix->width();
        if(++pass >= 3) pass = 0;
        matrix->setTextColor(colors[pass]);
    }
    matrix->show();
    delay(100);
}