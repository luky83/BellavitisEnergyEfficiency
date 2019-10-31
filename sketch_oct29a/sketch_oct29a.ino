//breaks 32 bit float into 4 bytes and reassembles into float
//representative of received data format

void setup() {
  Serial.begin(9600);
  float startingNumber = 12345.67;
  uint8_t recArray[4];

  memcpy (&recArray, (uint8_t*)&startingNumber, sizeof(startingNumber));

  for(uint8_t i=0;i<4;i++){
    Serial.print("0X");
    Serial.println(recArray[i],HEX);
  }
  //little endian byte order 0xAE,0xE6,0x40,0x46

  float value;
  memcpy(&value, recArray, sizeof(recArray));
  Serial.println(value);

}
void loop() {
}
