//sauna controller by Arvid&Marie 2021
//select  arduino nano "old bootloader" to be able to upload


//todo
//calibrate temperature correction
//make the heating with the heating elements being switched progressively
//double check safety loops, in case something crashes, how to ensure shutdown?
//clean up code and make setting apparent
//heating graph on oled?




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

#define ledPin 13

Bounce pushButton = Bounce();
Bounce doorSwitch = Bounce();


#define LED_PIN0    11
#define LED_PIN1    12
#define COLOR_ORDER BRG
#define CHIPSET     WS2811
#define NUM_LEDS_PER_STRIP 12
CRGB leds[NUM_LEDS_PER_STRIP * 2];


extern const TProgmemPalette16 saunaPalette1_p PROGMEM;

#define BRIGHTNESS  255
int ledNo = 0;


bool heatingOn = false;
bool firstTimeHot = false;


Adafruit_AM2315 am2315;


float desiredTemperature[] = {55., 60., 65., 70., 75., 80., 85., 90., 95., 100.};
int tempSettingAmount = 10;
int tempSetting = 2;
#define hysteresis .5//the amount of difference between turning on and off (system could be made more advance with PID
#define maxTemp 110

unsigned long timer;
unsigned long  maxTime = 60000 * 30; //= 30m
unsigned long timeToGetHot = 0;

int buttonPressTime = 1000;//ms

void heatingLoop();
void ledLoop();
void buttonLoop();

SchedTask HeatingTask (0, 1000, heatingLoop);              // define the turn on task (dispatch now, every 3 sec)
//SchedTask LedTask (300, 35, ledLoop);//16ms ~= 60fps 35~=30fps
SchedTask LedTask (300, 500, ledLoop);//run every .1s
SchedTask ButtonTask (600, 100, buttonLoop);              // define the turn on task (dispatch now, every 3 sec)


//OLED settings
#define SDA_PIN -1
#define SCL_PIN -1
#define RESET_PIN -1
#define OLED_ADDR -1
#define FLIP180 1
#define INVERT 0
#define USE_HW_I2C 1
SSOLED ssoled;
char charVal[10];
bool oledFunctional = true;

//float temp0adjust[] =   {13., 9., 65., 75.}; //sensor0 lowMeasured,lowReal,highMeasured,highReal
float temp0adjust[] =   {13., 13., 65., 65.}; //sensor0 lowMeasured,lowReal,highMeasured,highReal
float temp2adjust[] =   {13., 9., 65., 85.}; //sensor0 lowMeasured,lowReal,highMeasured,highReal





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
    oledWriteString(&ssoled, 0, 50, 7, (char *)"starting", FONT_NORMAL, 0, 1);
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

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

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


  pushButton.update();
  if (!pushButton.read()) {
    timerStartup();
  }

}

//this needs to be here so that the functions get loaded properly
typedef void (*SimplePatternList[])();
SimplePatternList ledPatterns = {timeProgram, red, green};
int currentLedPattern = 0;
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void loop() {
  SchedBase::dispatcher();
}

void heatingLoop() {



  if ((unsigned long)(millis() - timer) >= maxTime && firstTimeHot) {
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
    if (temperature2 = backupTempSensor.getTempCByIndex(0) == 85)temperature2 = 0; //85 is a false reading, so it's discarded


    temperature = fmap(temperature, temp0adjust[0], temp0adjust[2], temp0adjust[1], temp0adjust[3]);

    temperature2 = fmap(temperature2, temp2adjust[0], temp2adjust[2], temp2adjust[1], temp2adjust[3]);

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


      //make a little bar showing the heating level graphically
#define lineEnd 123
      int heatingLevel = map (temperature, 20, desiredTemperature[tempSetting % tempSettingAmount], 0, 7);
      for ( int i = 7; i >= 7 - heatingLevel; i--) {
        if (!firstTimeHot)    oledWriteString(&ssoled, 0,  lineEnd, i , (char *)"|", FONT_NORMAL, 0, 1);
        else     oledWriteString(&ssoled, 0,  lineEnd, i , (char *)"+", FONT_NORMAL, 0, 1);

      }



    }
    if (temperature > maxTemp || temperature2 > maxTemp) {
      digitalWrite(heaterRelayPin, LOW);
      Serial.println(F("overheated, stopping now"));
      heatingOn = false;
      setLeds(CRGB::Black);
      FastLED.show();
      oledFill(&ssoled, 0, 1);
      oledWriteString(&ssoled, 0,  0, 0, (char *)"OVERHEATED", FONT_NORMAL, 0, 1);
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

        if (!firstTimeHot) {
          timeToGetHot = millis();
          timer = millis();//start counting time from firt time to get hot
        }
        firstTimeHot = true;

      }
    }
  }
  //Serial.println("heatingLoop");
  backupTempSensor.requestTemperatures();
}



unsigned long buttonPressTimeStamp = 0;
void buttonLoop() {
  pushButton.update();
  if (pushButton.fell()) {
    Serial.print(F("button Pressed"));
    buttonPressTimeStamp = millis();
  }
  if (pushButton.rose()) {
    if ((millis() - buttonPressTimeStamp) > buttonPressTime) {

      currentLedPattern++;
      Serial.println(F("button long->change LED"));
      oledFill(&ssoled, 0, 1);
      oledWriteString(&ssoled, 0,  0, 3, (char *)"led pattern", FONT_NORMAL, 0, 1);
      dtostrf(currentLedPattern % ARRAY_SIZE(ledPatterns), 4, 1, charVal);
      oledWriteString(&ssoled, 0,  50, 4, charVal, FONT_NORMAL, 0, 1);
      delay(1000);
    }
    else {
      tempSetting++;
      Serial.println(F("button short->change temp"));
      oledFill(&ssoled, 0, 1);
      oledWriteString(&ssoled, 0,  0, 3, (char *)"temp setting", FONT_NORMAL, 0, 1);
      dtostrf(desiredTemperature[tempSetting % tempSettingAmount], 4, 1, charVal);
      oledWriteString(&ssoled, 0,  50, 4, charVal, FONT_NORMAL, 0, 1);
      delay(1000);
    }
    timer = millis();//reset the timer
  }

  doorSwitch.update();
  if (doorSwitch.rose()) {
    Serial.println(F("door opened, reset time"));
    timer = millis();//reset the timer
  }
  //Serial.println("buttonLoop");
}

void ledLoop() {
  ledPatterns[currentLedPattern % ARRAY_SIZE(ledPatterns) ]();
  if (heatingOn)  leds[0] = CRGB::Red;
  else leds[0] = CRGB::Grey;
  FastLED.show();
  //Serial.println("ledLoop");
  if (!firstTimeHot)digitalWrite(ledPin, !digitalRead(ledPin)); //blinking light indicator that maxTemp has not yet been reached
}

void timeProgram( )
{

  unsigned long colorIndex = map(millis() - timer, 0, 15 * 60000, 0, 255);
  dtostrf(colorIndex, 4, 1, charVal);
  oledWriteString(&ssoled, 0,  105, 0, charVal, FONT_NORMAL, 0, 1);
  if (colorIndex > 255) {
    setLeds(  saunaPalette1_p[0]);
  }
  else {
    for ( int i = NUM_LEDS_PER_STRIP * 2; i >= 0 ; i--) {
      leds[i] = ColorFromPalette( saunaPalette1_p, colorIndex + i, 255, LINEARBLEND);
    }
  }
}

void setLeds(CRGB color) {
  for ( int i = 0; i < NUM_LEDS_PER_STRIP * 2; i++) {
    leds[i] = color;
  }
}

void red() {

  for (int i = 0; i < NUM_LEDS_PER_STRIP ; i++) {
    leds[i] = CHSV(20, 200, map(i, 0, NUM_LEDS_PER_STRIP , 255, 150));
  }
  for (int i = 0; i < NUM_LEDS_PER_STRIP ; i++) {
    leds[i + NUM_LEDS_PER_STRIP] = CHSV(10, 200, map(i, 0, NUM_LEDS_PER_STRIP , 255, 150));
  }
}

void green() {

  for (int i = 0; i < NUM_LEDS_PER_STRIP * 2; i++) {
    leds[i] = CHSV(100, 200, map(i, 0, NUM_LEDS_PER_STRIP * 2, 255, 100));
  }
  for (int i = 0; i < NUM_LEDS_PER_STRIP * 2; i++) {
    leds[i + NUM_LEDS_PER_STRIP] = CHSV(110, 200, map(i, 0, NUM_LEDS_PER_STRIP * 2, 255, 100));
  }
}

const TProgmemPalette16 saunaPalette1_p PROGMEM =
{
  CRGB::Grey,
  CRGB::Linen ,
  CRGB::MediumAquamarine,
  CRGB::LavenderBlush ,

  CRGB::LightBlue ,
  CRGB::Red,
  CRGB::MediumSlateBlue ,
  CRGB::Orange,

  CRGB::LightCoral ,
  CRGB::Goldenrod ,
  CRGB::BlanchedAlmond ,
  CRGB::DarkOliveGreen ,

  CRGB::DarkMagenta ,
  CRGB::ForestGreen ,
  CRGB::DarkOrange ,
  CRGB::Grey,


};

void timerStartup() {
  unsigned long waitTime = 0;
  Serial.println(F("timerStartup"));
  oledFill(&ssoled, 0, 1);
  oledWriteString(&ssoled, 0,  50, 3, (char *)"Time delay", FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,  0, 4, (char *)"short press +30m", FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,  0, 5, (char *)"long press confirm", FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,  0, 6, (char *)"wait:", FONT_NORMAL, 0, 1);

  oledWriteString(&ssoled, 0,  50, 6, (char *)timeToString(waitTime ).c_str(), FONT_NORMAL, 0, 1);

  pushButton.update();
  while (true) {
    pushButton.update();
    if (pushButton.fell()) {
      Serial.print(F("button Pressed"));
      buttonPressTimeStamp = millis();
    }
    if (buttonPressTimeStamp) {
      if (pushButton.rose()) {
        if ((millis() - buttonPressTimeStamp) > buttonPressTime) {

          Serial.println(F("button long->timer set"));
          unsigned long waitTimer = millis();
          while (millis() < waitTimer + waitTime) {//waiting
            oledWriteString(&ssoled, 0,  50, 6, (char *)timeToString(waitTimer - millis() + waitTime ).c_str(), FONT_NORMAL, 0, 1);
            delay(250);
          }
          return;
        }
        else {
          Serial.println(F("button short->add 30m"));
          waitTime += 60000 * 30;

          oledWriteString(&ssoled, 0,  50, 6, (char *)timeToString(waitTime ).c_str(), FONT_NORMAL, 0, 1);

        }
        timer = millis();//reset t7he timer
      }
    }
  }
}

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

float fmap (float sensorValue, float sensorMin, float sensorMax, float outMin, float outMax)
{
  return (sensorValue - sensorMin) * (outMax - outMin) / (sensorMax - sensorMin) + outMin;
}
