#include <XBee.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

XBee xbee = XBee();

const int LSMSCLK = 8;
const int LSMMISO = 9;
const int LSMMOSI = 10;
const int LSMXMCS = 11;
const int LSMGCS = 12;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSMSCLK, LSMMISO, LSMMOSI, LSMXMCS, LSMGCS);


const int ACCEL_BIAS[3] = {-1215, 107, -500};
const double ACCEL_SENSITIVITY[3] = {16025, 16123, 16485};
const int ACCEL_SUPPLY = 7;

double accel[3];
double prevAccel[3] = {0, 0, 0};

const double pi = 3.14;

uint8_t payload[7];
Tx16Request tx = Tx16Request(0x1234, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

double pitch;
double roll;

//reach
const int DESIREDA_MIN = 15;
const int DESIREDA_MAX = 35;
const int DESIREDB_MIN = -15;
const int DESIREDB_MAX = 15;
const int DESIREDY_MIN = -10;
const int DESIREDY_MAX = 12;
const int DESIREDX_MIN = 4;
const int DESIREDX_MAX = 16;
const int DESIREDROT_MAX = 180;
const int DESIREDROT_MIN = 0;

volatile double desiredA;
volatile double desiredB;
volatile double desiredX;
volatile double desiredY;
volatile double desiredRot;    // 0: right, 180: left
volatile int claw;          // 0: open, 1:closed
volatile int moveDir;           // 0: stop, 1: forward, 2: reverse, 3: left, 4: right

const int FLEX_SENSOR_HIGH = A2;
const int FLEX_SENSOR_LOW = A1;
const int FLEX_SENSOR_PIN = A0;
const int STATUS_LED_PIN = 13;

int t = 0;

void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
}

void setup() {
  Serial.begin(9600);
  xbee.setSerial(Serial);
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(FLEX_SENSOR_HIGH, OUTPUT);
  pinMode(FLEX_SENSOR_LOW, OUTPUT);
  pinMode(ACCEL_SUPPLY, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(FLEX_SENSOR_HIGH, HIGH);
  digitalWrite(FLEX_SENSOR_LOW, LOW);
  digitalWrite(ACCEL_SUPPLY, HIGH);
  
  positionReset();
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    //Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    while(1);
  }
  
  delay(5000);

  lsm.read();
  prevAccel[0] = (lsm.accelData.x - ACCEL_BIAS[0]) / ACCEL_SENSITIVITY[0];
  prevAccel[1] = (lsm.accelData.y - ACCEL_BIAS[1]) / ACCEL_SENSITIVITY[1];
  prevAccel[2] = (lsm.accelData.z - ACCEL_BIAS[2]) / ACCEL_SENSITIVITY[2];
}

void loop() {
  //simulateControl();
  readSensors();
  calculatePosition();
  pack();
  xbee.send(tx);
  verifyResponse();
}

void positionReset() {
  desiredA = 25;
  desiredB = 0;
  desiredY = 5;
  desiredRot = 90;
  claw = 1;
  moveDir = 0;
}

void readSensors(){
  lsm.read();

  double a = 0.1;

  accel[0] = (lsm.accelData.x - ACCEL_BIAS[0]) / ACCEL_SENSITIVITY[0];
  accel[1] = (lsm.accelData.y - ACCEL_BIAS[1]) / ACCEL_SENSITIVITY[1];
  accel[2] = (lsm.accelData.z - ACCEL_BIAS[2]) / ACCEL_SENSITIVITY[2];

  for (int i = 0; i < 3; i++) {
    accel[i] = (1 - a) * prevAccel[i] + a * accel[i];
    prevAccel[i] = accel[i];
  }

  pitch = atan2(accel[1], -accel[2]) * 180 / pi;
  roll = atan2(-accel[0], sqrt(pow(accel[1],2) + pow(accel[2],2))) * 180 / pi;
//
//  Serial.print("Accel X: "); Serial.print(accel[0],4); Serial.print(" ");
//  Serial.print("Roll: "); Serial.print(roll);       Serial.print(" ");
//  Serial.print("Pitch: "); Serial.println(pitch);     Serial.print(" ");

  int flex = map(analogRead(FLEX_SENSOR_PIN), 240, 500, 0, 2);

  if (flex == 0) {
    positionReset();
  }
  claw = (flex <= 1) ? 1 : 0;
}

void calculatePosition() {
  if (abs(pitch) > 5) {
    desiredB = (pitch > 0) ? desiredB + pitch * 0.005 : desiredB + pitch * 0.005;
  }
  if (abs(roll) > 5) {
    desiredA = (roll > 0) ? desiredA - roll * 0.005 : desiredA - roll * 0.005;
  }
  desiredA = (desiredA > DESIREDA_MAX) ? DESIREDA_MAX : desiredA;
  desiredA = (desiredA < DESIREDA_MIN) ? DESIREDA_MIN : desiredA;
  desiredB = (desiredB > DESIREDB_MAX) ? DESIREDB_MAX : desiredB;
  desiredB = (desiredB < DESIREDB_MIN) ? DESIREDB_MIN : desiredB;

  desiredY = 0;

  desiredX = sqrt(pow(desiredA, 2) + pow(desiredB, 2)) - 13;
  desiredRot = atan2(desiredA, desiredB) * 180 / pi;

  desiredX = (desiredX > DESIREDX_MAX) ? DESIREDX_MAX : desiredX;
  desiredX = (desiredX < DESIREDX_MIN) ? DESIREDX_MIN : desiredX;
  desiredRot = (desiredRot > DESIREDROT_MAX) ? DESIREDROT_MAX : desiredRot;
  desiredRot = (desiredRot < DESIREDROT_MIN) ? DESIREDROT_MIN : desiredRot;
}

void pack() {
  payload[0] = (desiredX >= 0) ? 0 : 1;
  payload[1] = abs(int(desiredX * 10));
  payload[2] = (desiredY >= 0) ? 0 : 1;
  payload[3] = abs(int(desiredY * 10));
  payload[4] = int(desiredRot);
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
  double radius = 4;
  double deg = t * pi / 180;
  desiredX = radius * sin(deg * speed) + radius + 7;
  desiredY = -5;
  desiredRot = abs(int(t * speed) % 360 - 180) / 5 + 72;

  int flex = map(analogRead(FLEX_SENSOR_PIN), 240, 500, 0, 2);
  claw = (flex <= 1) ? 1 : 0;
}

