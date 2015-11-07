#include <XBee.h>

XBee xbee = XBee();

const double pi = 3.14;

uint8_t payload[7];
Tx16Request tx = Tx16Request(0x1234, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

volatile double desiredX;
volatile double desiredY;
volatile int desiredRot;    // 0: right, 180: left
volatile int claw;          // 0: open, 1:closed
volatile int moveDir;           // 0: stop, 1: forward, 2: reverse, 3: left, 4: right

const int FLEX_SENSOR_HIGH = A2;
const int FLEX_SENSOR_LOW = A1;
const int FLEX_SENSOR_PIN = A0;
const int STATUS_LED_PIN = 13;

int t = 0;

void setup() {
  Serial.begin(9600);
  xbee.setSerial(Serial);
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(FLEX_SENSOR_HIGH, OUTPUT);
  pinMode(FLEX_SENSOR_LOW, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(FLEX_SENSOR_HIGH, HIGH);
  digitalWrite(FLEX_SENSOR_LOW, LOW);
  
  positionReset();
  
  delay(5000);
}

void loop() {
  //simulateControl();
  calculatePosition();
  pack();
  xbee.send(tx);
  verifyResponse();
}

void positionReset() {
  desiredX = 0;
  desiredY = 5;
  desiredRot = 90;
  claw = 1;
  moveDir = 0;
}

void calculatePosition() {
  int flex = map(analogRead(FLEX_SENSOR_PIN), 240, 420, 0, 2);
  desiredX = 0;
  desiredY = 6;
  desiredRot = 90;
  if (flex == 0) {
    positionReset();
  }
  claw = (flex <= 1) ? 1 : 0;
}

void pack() {
  payload[0] = (desiredX >= 0) ? 0 : 1;
  payload[1] = abs(int(desiredX * 10));
  payload[2] = (desiredY >= 0) ? 0 : 1;
  payload[3] = abs(int(desiredY * 10));
  payload[4] = desiredRot;
  payload[5] = claw;
  payload[6] = moveDir;
}

void verifyResponse() {
  if (xbee.readPacket(5000)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS) {
        digitalWrite(STATUS_LED_PIN, HIGH);
      } else {
        digitalWrite(STATUS_LED_PIN, LOW);
      }
    } else {
      digitalWrite(STATUS_LED_PIN, LOW);
    }
  }
}

void simulateControl() {
  t++;
  double speed = 2;
  double radius = 8;
  double deg = t * pi / 180;
  claw = (t % 360 >= 180) ? 1 : 0;
  //claw = 1;
  desiredY = radius * sin(deg * speed) + 2;
  desiredX = 6;
  desiredRot = abs(int(t * speed) % 360 - 180) / 5 + 72;
}

