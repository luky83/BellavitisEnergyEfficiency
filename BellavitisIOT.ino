
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
int readings = 0;
int DHTerr = SimpleDHTErrSuccess;

AirQualitySensor airqualitysensor(A0);
int8_t current_quality = -1;

int window1_pin = 7;
uint8_t window1 = 0;


PacketSerial myPacketSerial;
SoftwareSerial mySerial(10, 11); // RX, TX
uint8_t myPacket[14];
bool ok_to_send = false; // semaphore to prevent sending data after resetting average


SoftwareSerial co2_sensor(8, 9);      // TX, RX
const unsigned char cmd_get_sensor[] = {0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
byte data[9];
int CO2_temperature = 0;
int CO2PPM = 0;

const long readInterval = 1000;           // interval at which to send data
const long sendInterval = 10000;           // interval at which to send data
unsigned long lastRead = 0;        // will store last time data was sent
unsigned long lastSent = 0;        // will store last time data was sent

void(* Reset)(void) = 0;

void setup() {
  Serial.begin(9600);
  mySerial.begin(38400);
  co2_sensor.begin(9600);
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
    readCO2Sensor();
    readReelSwitch();
    printDebugInfo();
    ok_to_send = true;
  }

  if (currentMillis - lastSent >= sendInterval && ok_to_send) {
    lastSent = currentMillis;
    memcpy (&myPacket[0], (uint8_t*)&temperature_avg, 4);
    memcpy (&myPacket[4], (uint8_t*)&CO2_temperature, 2);
    memcpy (&myPacket[6], (uint8_t*)&humidity_avg, 4);
    memcpy (&myPacket[10], (uint8_t*)&current_quality, 1);
    memcpy (&myPacket[11], (uint8_t*)&CO2PPM, 2);
    memcpy (&myPacket[13], (uint8_t*)&window1, 1);
    // Send the packet.
    myPacketSerial.send(myPacket, 14);
    resetAverage();
    ok_to_send = false;
  }
}

void runAverage() {
  readings++;
  temperature_avg += (temperature - temperature_avg) / readings;
  humidity_avg += (humidity - humidity_avg) / readings;
}

void resetAverage() {
  temperature_avg = 0;
  humidity_avg = 0;
  readings = 0;
}

void readAirQuality() {
  current_quality = airqualitysensor.slope();
}

void readDHT22() {
  // read without samples.
  // @remark We use read2 to get a float data, such as 10.1*C
  //    if user doesn't care about the accurate data, use read to get a byte data, such as 10*C.
  DHTerr = SimpleDHTErrSuccess;
  if ((DHTerr = dht22.read2(pinDHT22, &temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.println(DHTerr); delay(2000);
    Reset(); // reboot Arduino
  }
  runAverage();
}

void readCO2Sensor() {
  // send CO2 request packet
  for (int i = 0; i < sizeof(cmd_get_sensor); i++) {
    co2_sensor.write(cmd_get_sensor[i]);
  }
  delay(100);
  // read CO2 request response
  if (co2_sensor.available() == 9) {
    for (int i = 0; i < 9; i++) {
      data[i] = co2_sensor.read();
    }
  } else { // discard serial data
    Serial.println("CO2 packet incomplete, discarding buffer");
    while (co2_sensor.available()) {
      co2_sensor.read();
      delay(100);
    }
  }
  if ( CO2_sensor_checksum_ok()) {
    CO2PPM = (int)data[2] * 256 + (int)data[3];
    CO2_temperature = (int)data[4] - 40;
  } else {
    Serial.println("CO2 packet checksum fail, discarding packet");
    CO2PPM = -1;
    CO2_temperature = -300;
  }
}

void readReelSwitch() {
  window1 = digitalRead(window1_pin);
}

bool CO2_sensor_checksum_ok() {
  return (1 + (0xFF ^ (byte)(data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]))) == data[8];
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

  Serial.print("CO2 sensor Temperature: ");
  Serial.print(CO2_temperature);
  Serial.print("  CO2: ");
  Serial.print(CO2PPM);
  Serial.println("");


  Serial.print("Finestra 1: ");
  if (window1) Serial.println("aperta");
  else Serial.println("chiusa");
}
