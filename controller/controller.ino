#include <XBee.h>

XBee xbee = XBee();

const double pi = 3.14;

uint8_t payload[6];
Tx16Request tx = Tx16Request(0x1234, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

volatile double desiredX;
volatile double desiredY;
volatile int desiredRot;
volatile int claw;

int statusLed = 13;

int t = 0;

void setup() {
  pinMode(statusLed, OUTPUT);
  digitalWrite(statusLed, LOW);
  
  Serial.begin(9600);
  xbee.setSerial(Serial);
  
  positionReset();
  
  delay(5000);
}

void loop() {
  simulateControl();
  //calculatePosition();
  pack();
  xbee.send(tx);
  verifyResponse();
}

void positionReset() {
  desiredX = 0;
  desiredY = 5;
  desiredRot = 90;
}

void calculatePosition() {
  desiredX = 0;
  desiredY = 5;
  desiredRot = 90;
  claw = 0;
}

void pack() {
  payload[0] = (desiredX >= 0) ? 0 : 1;
  payload[1] = abs(int(desiredX * 10));
  payload[2] = (desiredY >= 0) ? 0 : 1;
  payload[3] = abs(int(desiredY * 10));
  payload[4] = desiredRot;
  payload[5] = claw;
}

void verifyResponse() {
  if (xbee.readPacket(5000)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS) {
        digitalWrite(statusLed, HIGH);
      } else {
        digitalWrite(statusLed, LOW);
      }
    } else {
      digitalWrite(statusLed, LOW);
    }
  }
}

void simulateControl() {
  t++;
  double speed = 2;
  double radius = 9;
  double deg = t * pi / 180;
  claw = (t % 360 >= 180) ? 1 : 0;
  desiredY = radius * sin(deg * speed) + 2;
  desiredX = 6;
  desiredRot = abs(int(t * speed) % 360 - 180) / 6 + 75;
}

