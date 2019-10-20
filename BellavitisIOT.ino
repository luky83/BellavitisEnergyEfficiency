#include <SimpleDHT.h>
// for DHT22,
//      VCC: 5V or 3V
//      GND: GND
//      DATA: 2
int pinDHT22 = 2;
SimpleDHT22 dht22;

#include"AirQuality.h"
#include"Arduino.h"
AirQuality airqualitysensor;
int current_quality = -1;

#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  Serial.begin(9600);
  airqualitysensor.init(A0);
  
  mySerial.begin(4800);
}

void loop() {
  current_quality = airqualitysensor.slope();
  if (current_quality >= 0) { // if a valid data returned.
    // read without samples.
    // @remark We use read2 to get a float data, such as 10.1*C
    //    if user doesn't care about the accurate data, use read to get a byte data, such as 10*C.
    float temperature = 0;
    float humidity = 0;
    int err = SimpleDHTErrSuccess;
    if ((err = dht22.read2(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
      Serial.print("Read DHT22 failed, err="); Serial.println(err); delay(2000);
      return;
    }
    Serial.println();
    Serial.println("=================================");
    Serial.print((float)temperature); Serial.print(" *C, ");
    Serial.print((float)humidity); Serial.println(" RH%");
    if (current_quality == 0)
      Serial.println("High pollution! Force signal active");
    else if (current_quality == 1)
      Serial.println("High pollution!");
    else if (current_quality == 2)
      Serial.println("Low pollution!");
    else if (current_quality == 3)
      Serial.println("Fresh air");

    mySerial.print((float)temperature);
    mySerial.print(",");
    mySerial.print((float)humidity);
    mySerial.print(",");
    mySerial.print(current_quality);
    mySerial.print("\n");
  }
}

ISR(TIMER2_OVF_vect)
{
  if (airqualitysensor.counter == 122) //set 2 seconds as a detected duty
  {
    airqualitysensor.last_vol = airqualitysensor.first_vol;
    airqualitysensor.first_vol = analogRead(A0);
    airqualitysensor.counter = 0;
    airqualitysensor.timer_index = 1;
    PORTB = PORTB ^ 0x20;
  }
  else
  {
    airqualitysensor.counter++;
  }
}
