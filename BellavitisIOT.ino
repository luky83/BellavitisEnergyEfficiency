
#include"Arduino.h"
#include <SimpleDHT.h>
#include"Air_Quality_Sensor.h"
#include <SoftwareSerial.h>
#include <PacketSerial.h>

int pinDHT22 = 2;
SimpleDHT22 dht22;
float temperature = 0;
float humidity = 0;
float temperature_avg = 0;
float humidity_avg = 0;
int readings = 1;

AirQualitySensor airqualitysensor(A0);
int8_t current_quality = -1;

int window1_pin = 7;
uint8_t window1 = 0;


PacketSerial myPacketSerial;
SoftwareSerial mySerial(10, 11); // RX, TX
uint8_t myPacket[10];
bool ok_to_send = false; // semaphore to prevent sending data after resetting average

unsigned long lastRead = 0;        // will store last time data was sent
const long readInterval = 1000;           // interval at which to send data
unsigned long lastSent = 0;        // will store last time data was sent
const long sendInterval = 10000;           // interval at which to send data


void setup() {
  Serial.begin(9600);
  mySerial.begin(38400);
  myPacketSerial.setStream(&mySerial);

  while (!Serial);
  Serial.println("Waiting sensor to init...");
  delay(20000);
  if (airqualitysensor.init()) {
    Serial.println("Sensor ready.");
  }
  else {
    Serial.println("Sensor ERROR!");
  }

  pinMode(window1_pin, INPUT_PULLUP);
}

unsigned long currentMillis;
void loop() {
  myPacketSerial.update();

  currentMillis = millis();
  if (currentMillis - lastRead >= readInterval) {
    lastRead = currentMillis;
    readAirQuality();
    readDHT22();
    readWindows();
    printDebugInfo();
    ok_to_send = true;
  }

  currentMillis = millis();
  if (currentMillis - lastSent >= sendInterval && ok_to_send) {
    lastSent = currentMillis;
    memcpy (&myPacket[0], (uint8_t*)&temperature_avg, 4);
    memcpy (&myPacket[4], (uint8_t*)&humidity_avg, 4);
    memcpy (&myPacket[8], (uint8_t*)&current_quality, 1);
    memcpy (&myPacket[9], (uint8_t*)&window1, 1);
    // Send the packet.
    myPacketSerial.send(myPacket, 10);
    resetAverage();
    ok_to_send = false;
  }

}

void runAverage() {
  temperature_avg += (temperature - temperature_avg) / readings;
  humidity_avg += (humidity - humidity_avg) / readings;
  readings++;
}

void resetAverage() {
  float temperature_avg = 0;
  float humidity_avg = 0;
  int readings = 1;
}

void readAirQuality() {
  current_quality = airqualitysensor.slope();
}

void readDHT22() {
  // read without samples.
  // @remark We use read2 to get a float data, such as 10.1*C
  //    if user doesn't care about the accurate data, use read to get a byte data, such as 10*C.
  int err = SimpleDHTErrSuccess;
  if ((err = dht22.read2(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.println(err); delay(2000);
    return;
  }
  runAverage();
}

void readWindows() {
  window1 = digitalRead(window1_pin);
}

void printDebugInfo() {
  Serial.println();
  Serial.println("=================================");
  Serial.print((float)temperature); Serial.print(" *C, ");
  Serial.print((float)humidity); Serial.print(" RH%, ");

  if (current_quality >= 0) { // if a valid data returned.
    Serial.print("Air Quality Sensor value: ");
    Serial.println(airqualitysensor.getValue());
    if (current_quality == AirQualitySensor::FORCE_SIGNAL)
      Serial.println("High pollution! Force signal active");
    else if (current_quality == AirQualitySensor::HIGH_POLLUTION)
      Serial.println("High pollution!");
    else if (current_quality == AirQualitySensor::LOW_POLLUTION)
      Serial.println("Low pollution!");
    else if (current_quality == AirQualitySensor::FRESH_AIR)
      Serial.println("Fresh air");
  } else {
   Serial.println("Air Quality sensor returned no valid data");
  }
  Serial.print("Finestra1: ");
  if(window1) Serial.println("aperta");
  else Serial.println("chiusa");
}
