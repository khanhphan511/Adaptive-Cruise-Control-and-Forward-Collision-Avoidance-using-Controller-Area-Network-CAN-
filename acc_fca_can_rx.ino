#define ENCODEROUTPUT 540
#define PWM 9
#define encoder0pinA 3

#include "mcp_can.h"

unsigned char buf[1];
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

int M_left = 4;
int E_left = 5;
int rpm = 0;
volatile unsigned long encoderValue = 0;
unsigned long lastmillis = 0;

void setup() {
  Serial.begin(9600);

  pinMode(M_left, OUTPUT);
  pinMode(E_left, OUTPUT);
  pinMode(encoder0pinA, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);

  digitalWrite(M_left, LOW);
  digitalWrite(E_left, HIGH);

  // add interrupt
  attachInterrupt(digitalPinToInterrupt(encoder0pinA), updateEncoder, FALLING);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
}

void loop() {
  unsigned char len = 0;
  float v;
  float pi = 3.14159265;

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();

    Serial.println("-------------------------");

    int value = (int)buf[0];

    if (value >= 12 && value <= 25) {
      int val_output = map(value, 12, 25, 0, 255);

      while (millis() - lastmillis != 1000) {
        analogWrite(PWM, val_output);
      }

      // disable interrupt when calculating
      // detachInterrupt(digitalPinToInterrupt(encoder0pinA));
      noInterrupts();

      rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);
      Serial.print("rpm = ");
      Serial.println(rpm);

      Serial.print("Linear Velocity (V) = ");
      // add equation to calc linear velocity, wheel radius = 3cm
      Serial.print(v = (2.0f * pi * 0.03f) * (rpm / 60.0f));
      Serial.println("m/s");

      Serial.print("PWM =");
      Serial.println(value);

      encoderValue = 0;
      lastmillis = millis();

      // attachInterrupt(digitalPinToInterrupt(encoder0pinA), updateEncoder, FALLING);
      interrupts();
    } else {
      Stop();
    }
  }
}

void updateEncoder() {
  // add encoderValue by 1, each time it detect rising signal from hall sensorA
  encoderValue++;
}

void Stop() {  // Motor stops
  digitalWrite(PWM, LOW);
}
