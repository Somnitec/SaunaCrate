//sauna controller by Arvid&Marie 2021

//select  arduino nano "old bootloader"
#include <Arduino.h>
#include "Adafruit_Si7021.h"
#include <Bounce2.h>
#include <FastLED.h>
#include <SchedTask.h>

#define heaterRelayPin A2
#define buttonPin A0

#define LED_PIN0    9
#define LED_PIN1    3
#define COLOR_ORDER BRG
#define CHIPSET     WS2811
#define NUM_LEDS_PER_STRIP 12
CRGB leds[NUM_LEDS_PER_STRIP * 2];

#define FRAMES_PER_SECOND 60
#define BRIGHTNESS  255
int ledNo = 0;

Bounce debouncer = Bounce();
Adafruit_Si7021 sensor = Adafruit_Si7021();

int buttonState = 0;
bool heatingOn = false;
bool firstTimeHot = false;

#define desiredTemperature 85
#define hysteresis .5//the amount of difference between turning on and off (system could be made more advance with PID
#define maxTemp 110

unsigned long timer;
int maxTime = 1000 * 60 * 60 * 2; //= 2 hours

void heatingLoop();
void ledLoop();
void buttonLoop();

SchedTask HeatingTask (0, 1000, heatingLoop);              // define the turn on task (dispatch now, every 3 sec)
//SchedTask LedTask (300, 1000 / FRAMES_PER_SECOND, ledLoop);         // define the turn off task (dispatch in 1 sec, every 3 sec)
SchedTask LedTask (300, 35, ledLoop);//16ms ~= 60fps 35~=30fps
SchedTask ButtonTask (600, 100, buttonLoop);              // define the turn on task (dispatch now, every 3 sec)

TBlendType    currentBlending = LINEARBLEND;
extern const TProgmemPalette16 saunaPalette1_p PROGMEM;


void setup() {
  pinMode(heaterRelayPin, OUTPUT);
  digitalWrite(heaterRelayPin, LOW);

  pinMode(buttonPin, INPUT_PULLUP);
  debouncer.attach(buttonPin);
  debouncer.interval(30); // interval in ms

  Serial.begin(115200);

  if (!sensor.begin()) {
    Serial.println("Did not find Si7021 sensor!");
    while (true)
      ;
  }

  FastLED.addLeds<CHIPSET, LED_PIN0, COLOR_ORDER>(leds, 0, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<CHIPSET, LED_PIN1, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );

  FastLED.setBrightness( BRIGHTNESS );

  Serial.println("Started");
  timer = millis();

}

void loop() {

  SchedBase::dispatcher();
}

void heatingLoop() {


  float temperature = sensor.readTemperature();
  float humidity = sensor.readHumidity();
  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.print("\tTemperature:");
  Serial.print(temperature);
  Serial.print("\tDesiredTemperature:");
  Serial.println(desiredTemperature);

  if ((unsigned long)(millis() - timer) >= maxTime) {
    digitalWrite(heaterRelayPin, LOW);
    Serial.println("time's up, stopping now");
    heatingOn = false;
    //while (true);//make this respond to a button press to restart again
  }

  else {
    if (temperature > maxTemp) {
      digitalWrite(heaterRelayPin, LOW);
      Serial.println("overheated, stopping now");
      heatingOn = false;
      while (true);
    }



    if (!heatingOn) {
      if (temperature < desiredTemperature - hysteresis) {
        digitalWrite(heaterRelayPin, HIGH);
        Serial.print("turning on heat, desired temp:");
        Serial.println(desiredTemperature);
        heatingOn = true;
      }
    }
    else if (heatingOn) {
      if (temperature > desiredTemperature) {
        digitalWrite(heaterRelayPin, LOW);
        Serial.println("turning off heat, desired temp:");
        Serial.println(desiredTemperature);
        heatingOn = false;
        firstTimeHot = true;
      }
    }

    //Serial.println("heatingLoop");
  }
}


void ledLoop() {
  /*
    //test leds
    leds[ledNo % (NUM_LEDS_PER_STRIP * 2)] = CRGB::Black;
    ledNo++;
    leds[ledNo % (NUM_LEDS_PER_STRIP * 2)] = CRGB::White;
  */
  if (firstTimeHot) {
    static uint8_t startIndex = 0;
    startIndex = startIndex + 1;
    FillLEDsFromPaletteColors( startIndex);
  } else {
    for ( int i = 0; i < NUM_LEDS_PER_STRIP * 2; i++) {
      leds[i] = CRGB::Blue;
    }
  }
  FastLED.show();

  //Serial.println("ledLoop");

}

void buttonLoop() {
  debouncer.update();
  if (debouncer.fell()) {
    buttonState++;
    Serial.print("button Pressed:");
    Serial.println(buttonState);

    timer = millis();//reset the timer
  }

  //Serial.println("buttonLoop");

}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
  uint8_t brightness = 255;

  for ( int i = 0; i < NUM_LEDS_PER_STRIP * 2; i++) {
    leds[i] = ColorFromPalette( saunaPalette1_p, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}

/*
  const TProgmemPalette16 saunaPalette1_p PROGMEM =
  {
  CRGB::Red,
  CRGB::Gray, // 'white' is too bright compared to red and blue
  CRGB::Blue,
  CRGB::Black,

  CRGB::Red,
  CRGB::Gray,
  CRGB::Green,
  CRGB::Black,

  CRGB::Red,
  CRGB::Red,
  CRGB::Green,
  CRGB::Green,

  CRGB::Blue,
  CRGB::Blue,
  CRGB::Black,
  CRGB::Black
  };
*/
const TProgmemPalette16 saunaPalette1_p PROGMEM =
{
  CRGB::Red,
  CRGB::Red,
  CRGB::Red,
  CRGB::Red,

  CRGB::Red,
  CRGB::Red,
  CRGB::Red,
  CRGB::Red,

  CRGB::Orange,
  CRGB::Orange,
  CRGB::Orange,
  CRGB::Orange,

  CRGB::Orange,
  CRGB::Orange,
  CRGB::Orange,
  CRGB::Orange,


};
