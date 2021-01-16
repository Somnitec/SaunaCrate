#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS A2

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

float Celcius = 0;
void setup(void)
{

  Serial.begin(115200);
  Serial.println("starting");
  sensors.begin();
  sensors.setResolution(9);
  Serial.println("sensor started");
}

void loop(void)
{
  sensors.requestTemperatures();
  delay(1000);
  Celcius = sensors.getTempCByIndex(0);
  
  Serial.print("C:");
  Serial.println(Celcius);

}
