#include <mcp_can.h>
#include <SPI.h>
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
#else
  #define SERIAL Serial
#endif
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);
const int bufferSize = 40;
int buffer[bufferSize];
int index;
int IRin = 0;
unsigned char distance;
void setup() {
  pinMode (IRin,INPUT);
  Serial.begin(9600);
  while(CAN_OK != CAN.begin(CAN_500KBPS)){
    SERIAL.println("CAN BUS Shield init fail");
    SERIAL.println(" Init CAN BUS Shield again");
    delay(100);
  }
  SERIAL.println("CAN BUS Shield init ok!");
}


void loop() {
  int a=analogRead(IRin);
  Serial.print(a); Serial.print("\t");
  int Range = ((6787/(a+3))-4);
  Serial.println(average());
  buffer[index] = Range;
  index++;
  if (index >=  bufferSize) index = 0;
  distance = average();
  CAN.sendMsgBuf(0x07, 0, 1, distance);
  delay(10);
}
int average(){
  long sum = 0;
  for (int i = 0; i < bufferSize; i++){
    sum += buffer[i];
  }
  return (int)(sum / bufferSize);
}





