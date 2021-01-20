//sauna controller by Arvid&Marie 2021

//display warm up time
//warm up - 1h wait, 30m after each door
//see how to fix backupsensor
//button press is change color, long press is change temp

//select  arduino nano "old bootloader"
#include <Arduino.h>
#include <Bounce2.h>
#include <FastLED.h>
#include <SchedTask.h>
#include <ss_oled.h>
#include <Wire.h>
#include <Adafruit_AM2315.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS A2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature backupTempSensor(&oneWire);

#define heaterRelayPin 2
#define buttonGndPin A1
#define buttonPin A0
#define doorPin 10

Bounce pushButton = Bounce();
Bounce doorSwitch = Bounce();


#define LED_PIN0    11
#define LED_PIN1    12
#define COLOR_ORDER BRG
#define CHIPSET     WS2811
#define NUM_LEDS_PER_STRIP 12
CRGB leds[NUM_LEDS_PER_STRIP * 2];

#define FRAMES_PER_SECOND 60
#define BRIGHTNESS  255
int ledNo = 0;

int buttonState = 0;
bool heatingOn = false;
bool firstTimeHot = false;


Adafruit_AM2315 am2315;


float desiredTemperature[] = {45., 55., 65., 75., 85., 95.};
int tempSettingAmount = 6;
int tempSetting = 3;
#define hysteresis .5//the amount of difference between turning on and off (system could be made more advance with PID
#define maxTemp 110

unsigned long timer;
unsigned long  maxTime = 60000 * 30; //= 30m
unsigned long timeToGetHot = 0;

void heatingLoop();
void ledLoop();
void buttonLoop();

SchedTask HeatingTask (0, 1000, heatingLoop);              // define the turn on task (dispatch now, every 3 sec)
//SchedTask LedTask (300, 1000 / FRAMES_PER_SECOND, ledLoop);         // define the turn off task (dispatch in 1 sec, every 3 sec)
SchedTask LedTask (300, 35, ledLoop);//16ms ~= 60fps 35~=30fps
SchedTask ButtonTask (600, 100, buttonLoop);              // define the turn on task (dispatch now, every 3 sec)

TBlendType    currentBlending = LINEARBLEND;
extern const TProgmemPalette16 saunaPalette1_p PROGMEM;

//OLED settings
#define SDA_PIN -1
#define SCL_PIN -1
#define RESET_PIN -1
#define OLED_ADDR -1
#define FLIP180 0
#define INVERT 0
#define USE_HW_I2C 1
SSOLED ssoled;

bool oledFunctional = true;

void setup() {
  Serial.begin(115200);
  Serial.println(F("initializing"));
  pinMode(heaterRelayPin, OUTPUT);
  digitalWrite(heaterRelayPin, LOW);

  Serial.println(F("heaterpin set"));
  int rc;
  rc = oledInit(&ssoled, OLED_128x64, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L);       // Standard HW I2C bus at 400Khz
  if (rc != OLED_NOT_FOUND)
  {
    Serial.println(F("oled started"));
    oledFill(&ssoled, 0, 1);
    oledWriteString(&ssoled, 0, 0, 0, (char *)"start", FONT_NORMAL, 0, 1);
    //delay(2000);
  }
  else
  {
    Serial.println(F("oled not found"));
    oledFunctional = false;
    //while (1) {};//oled not working
  }

  pinMode(buttonGndPin, OUTPUT);
  digitalWrite(buttonGndPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  pushButton.attach(buttonPin);
  pushButton.interval(30); // interval in ms


  pinMode(doorPin, INPUT_PULLUP);
  doorSwitch.attach(doorPin);
  doorSwitch.interval(30); // interval in ms



  if (! am2315.begin()) {
    Serial.println(F("am2315 sensor not found, check wiring & pullups!"));
    while (1);
  }
  Serial.println(F("am2315 started"));

  backupTempSensor.begin();
  backupTempSensor.setResolution(9);
  Serial.println(F("backup tempsensor started"));

  FastLED.addLeds<CHIPSET, LED_PIN0, COLOR_ORDER>(leds, 0, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<CHIPSET, LED_PIN1, COLOR_ORDER>(leds, NUM_LEDS_PER_STRIP, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );

  FastLED.setBrightness( BRIGHTNESS );

  Serial.println(F("leds started"));
  Serial.println(F("all initialized!"));
  timer = millis();


}

void loop() {
  SchedBase::dispatcher();
}

void heatingLoop() {



  if ((unsigned long)(millis() - timer) >= maxTime) {
    digitalWrite(heaterRelayPin, LOW);
    Serial.println(F("time's up, stopping now"));
    heatingOn = false;
    firstTimeHot = false;
  }
  else {
    float temperature, temperature2, humidity;

    if (! am2315.readTemperatureAndHumidity(&temperature, &humidity)) {
      //Serial.println("Failed to read data from AM2315");
      return;
    }
    temperature2 = backupTempSensor.getTempCByIndex(0);

    Serial.print(F("\tTemperature:"));
    Serial.print(temperature);
    Serial.print(F("\tTemperature2:"));
    Serial.print(temperature2);
    Serial.print(F("\tDesiredTemperature:"));
    Serial.print(desiredTemperature[tempSetting % tempSettingAmount]);
    Serial.print(F("\tHeatingNow:"));
    Serial.print(heatingOn);
    Serial.print(F("\tHumidity:"));
    Serial.print(humidity);
    Serial.print(F("\ttime:"));
    Serial.print( timeToString(millis() ));
    Serial.print(F("\ttimeLeft:"));
    Serial.print( timeToString(maxTime - (millis() - timer) ));
    Serial.print(F("\ttimeToGetHot:"));
    Serial.println( timeToString(timeToGetHot));

    if (oledFunctional) {
#define txtOffset 50
    char charVal[10];

      oledFill(&ssoled, 0, 1);

      dtostrf(temperature, 4, 1, charVal);
      oledWriteString(&ssoled, 0,  0, 0, (char *)"temp:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset, 0, (char *)charVal, FONT_NORMAL, 0, 1);
      dtostrf(temperature2, 4, 1, charVal);
      oledWriteString(&ssoled, 0,  0, 1, (char *)"temp2:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset, 1, (char *)charVal, FONT_NORMAL, 0, 1);
      dtostrf(desiredTemperature[tempSetting % tempSettingAmount], 4, 1, charVal);
      oledWriteString(&ssoled, 0,  0, 2, (char *)"target:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset, 2, (char *)charVal, FONT_NORMAL, 0, 1);
      dtostrf(heatingOn, 1, 0, charVal);
      oledWriteString(&ssoled, 0,  0, 3, (char *)"heating:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset + 30, 3, (char *)charVal, FONT_NORMAL, 0, 1);
      dtostrf(humidity, 4, 1, charVal);
      oledWriteString(&ssoled, 0,  0, 4, (char *)"hum:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset, 4, (char *)charVal, FONT_NORMAL, 0, 1);

      oledWriteString(&ssoled, 0,  0, 5, (char *)"time:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset, 5, (char *)timeToString(millis()).c_str(), FONT_NORMAL, 0, 1);

      oledWriteString(&ssoled, 0,  0, 6, (char *)"left:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset, 6, (char *)timeToString(maxTime - (millis() - timer) ).c_str(), FONT_NORMAL, 0, 1);

      oledWriteString(&ssoled, 0,  0, 7, (char *)"tHot:", FONT_NORMAL, 0, 1);
      oledWriteString(&ssoled, 0,  txtOffset, 7, (char *)timeToString(timeToGetHot ).c_str(), FONT_NORMAL, 0, 1);


    }
    if (temperature > maxTemp) {
    digitalWrite(heaterRelayPin, LOW);
      Serial.println(F("overheated, stopping now"));
      heatingOn = false;
      setLeds(CRGB::Black);
      FastLED.show();
      while (true);
    }

    if (!heatingOn) {
    if (temperature < desiredTemperature[tempSetting % tempSettingAmount] - hysteresis) {
        digitalWrite(heaterRelayPin, HIGH);
        Serial.print(F("turning on heat, desiredTemperature:"));
        Serial.println(desiredTemperature[tempSetting % tempSettingAmount]);
        heatingOn = true;
      }
    }
    else if (heatingOn) {
    if (temperature > desiredTemperature[tempSetting % tempSettingAmount]) {
        digitalWrite(heaterRelayPin, LOW);
        Serial.println(F("turning off heat, desired temp:"));
        Serial.println(desiredTemperature[tempSetting % tempSettingAmount]);
        heatingOn = false;

        if (!firstTimeHot)timeToGetHot = millis();
        firstTimeHot = true;

      }
    }
  }
  //Serial.println("heatingLoop");
  backupTempSensor.requestTemperatures();
}

void ledLoop() {
  /*
    //test leds
    leds[ledNo % (NUM_LEDS_PER_STRIP * 2)] = CRGB::Black;
    ledNo++;
    leds[ledNo % (NUM_LEDS_PER_STRIP * 2)] = CRGB::White;
  */
  //if (firstTimeHot) {
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1;
  FillLEDsFromPaletteColors( startIndex);
  //} else {
  //  setLeds(CRGB::Blue);
  // }
  FastLED.show();
  //Serial.println("ledLoop");
}

void buttonLoop() {
  pushButton.update();
  if (pushButton.fell()) {
    buttonState++;
    Serial.print(F("button Pressed:"));
    Serial.println(buttonState);
    tempSetting++;

    timer = millis();//reset the timer
  }

  doorSwitch.update();
  if (doorSwitch.rose()) {
    Serial.println(F("door opened, reset time"));
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

void setLeds(CRGB color) {
  for ( int i = 0; i < NUM_LEDS_PER_STRIP * 2; i++) {
    leds[i] = color;
  }
}

const TProgmemPalette16 saunaPalette1_p PROGMEM =
{
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
  CRGB::Orange,


};

String timeToString(unsigned long value) {
  String string = "";
  unsigned long  secondPart = value / 1000 ;
  unsigned long  minutePart = secondPart / 60;
  unsigned long  hourPart = minutePart / 60;
  string += hourPart;
  string += ':';
  if (minutePart % 60 < 10)string += '0';
  string += minutePart % 60;
  string += ':';
  if (secondPart % 60 < 10)string += '0';
  string += secondPart % 60;
  return string;
}
