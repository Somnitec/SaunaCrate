
#include <FastLED.h>


#define LED_PIN0    11
#define LED_PIN1    12
#define COLOR_ORDER BRG
#define CHIPSET     WS2811
#define NUM_LEDS_PER_STRIP 12
CRGB leds[NUM_LEDS_PER_STRIP * 2];

#define FRAMES_PER_SECOND 100

uint32_t period0 = 10000L;
uint32_t period1 = 10555L;

void setup() {
  FastLED.addLeds<CHIPSET, LED_PIN0, COLOR_ORDER>(leds, 0, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<CHIPSET, LED_PIN1, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );

}

void loop() {
  int time0 = millis() % period0;              // returns a value between 0 and 4999;
  int time1 = millis() % period1;
  float angle0 = (PI * time0) / period0;        // mapping to degrees
  float angle1 = (PI * time1) / period1;
  float mod0 = sin(angle0);
  float mod1 =  sin(angle1);

  for (int i = 0; i < NUM_LEDS_PER_STRIP ; i++) {
    leds[i] = CHSV(fmap(mod0, 0, 1, 10, 50), 255, map(i, 0, NUM_LEDS_PER_STRIP , 255, 150));
  }
  for (int i = 0; i < NUM_LEDS_PER_STRIP ; i++) {
    leds[i + NUM_LEDS_PER_STRIP] = CHSV(fmap(mod0, 0, 1, 10, 50), 255, map(i, 0, NUM_LEDS_PER_STRIP , 255, 150));
  }

  FastLED.show();
  FastLED.delay(1000 / FRAMES_PER_SECOND);
}


float fmap (float sensorValue, float sensorMin, float sensorMax, float outMin, float outMax)
{
  return (sensorValue - sensorMin) * (outMax - outMin) / (sensorMax - sensorMin) + outMin;
}
