/**
 * EECS 149 Fa-15 Final Project
 * Real-time Remote Controlled Robotic Arm
 * Authors: Iraida Ermakova, Canruo Ying
 * 
 * This file is for the remote controller (user side).
 */

#include <XBee.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

//================ accelerometer constants ==========

// accelerometer pins
const int ACCEL_SUPPLY = 12;
const int LSMSCLK = 11;
const int LSMMISO = 7;
const int LSMMOSI = 10;
const int LSMXMCS = 8;
const int LSMGCS = 9;
// accelerometer bias/sensitivity
const int ACCEL_BIAS[3] = {-1215, 107, -500};
const double ACCEL_SENSITIVITY[3] = {16025, 16123, 16485};

//================ flex sensor constants ============

// flex sensor pins
const int FLEX_SENSOR_HIGH[3] = {6, 5, 4};
const int FLEX_SENSOR_LOW[3] = {A0, A2, A4};
const int FLEX_SENSOR_PIN[3] = {A1, A3, A5};

//================ calculation constants ============

// reach restrictions
const int DESIREDA_MIN = 18;
const int DESIREDA_MAX = 28;
const int DESIREDB_MIN = -10;
const int DESIREDB_MAX = 10;
const int DESIREDY_MIN = -5;
const int DESIREDY_MAX = 11;
const int DESIREDCLAWROT_MIN = 0;
const int DESIREDCLAWROT_MAX = 180;
// armature lengths for servos 12
const double ARM_12_LENGTH = 13;
// pi
const double pi = 3.14;

//================ XBee constants ===================

// status light pin
const int STATUS_LED_PIN = 13;

//================ XBee variables ===================

XBee xbee = XBee();
uint8_t payload[8];
Tx16Request tx = Tx16Request(0x1234, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();

//================ accelerometer variables ==========

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSMSCLK, LSMMISO, LSMMOSI, LSMXMCS, LSMGCS);
// accelerometer data
volatile double accel[3];
// previous accelerometer data for filtering
volatile double prevAccel[3] = {0, 0, 0};
// pitch/roll angles
double pitch;
double roll;

//================ flex sensor variables ============

// flex sensor data
double flex[3];
// previous sensor data for filtering
double prevFlex[3];

//================ servo/motor variables ============

// desired coordinates
volatile double desiredA;      // pos: forward, neg: aft
volatile double desiredB;      // pos: starboard, neg: port
volatile double desiredX;      // pos: forward (minus arm 12 length from A), neg: aft
volatile double desiredY;      // pos: above, neg: below
volatile double desiredRot;    // 0: starboard, 180: port
volatile double desiredClawRot;// 0: servo above, 180: servo below
volatile int claw;             // 0: open, 1:closed
volatile int moveDir;          // 0: stop, 1: forward, 2: reverse, 3: left, 4: right

//================ simulation variables =============

// time
int t = 0;

void setupSensor()
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
}

//================ setup ============================

/* Set up & initialize pins, check accelerometer status,
 * reset servo positions.
 */
void setup() {
  Serial.begin(9600);
  xbee.setSerial(Serial);

  // initialize XBee and accelerometer pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ACCEL_SUPPLY, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ACCEL_SUPPLY, HIGH);

  // initialize flex sensor pins
  for (int i = 0; i < 3; i++) {
    pinMode(FLEX_SENSOR_HIGH[i], OUTPUT);
    pinMode(FLEX_SENSOR_LOW[i], OUTPUT);
    digitalWrite(FLEX_SENSOR_HIGH[i], HIGH);
    digitalWrite(FLEX_SENSOR_LOW[i], LOW);
  }

  // reset servo positions to origin
  positionReset();

  // check accelerometer status
  if(!lsm.begin())
  {
    while(1);
  }

  // delay for XBee startup
  delay(2000);
}

//================ main loop ========================

/* Retrieve readings from sensors, calculate desired 
 * coordinates/servo positions, and then pack and send 
 * data.
 */
void loop() {
  //demo1();

  // get sensor readings
  readSensors();

  // calculate desired coordinates based on sensor readings
  calculatePosition();

  // pack all commands to be sent
  pack();

  // send data
  xbee.send(tx);

  // verify data has been received and display status through LED
  verifyResponse();
}

//================ reset arm position ===============

/* Reset position of all servos and command variables
 * to a safe/known region (origin).
 */
void positionReset() {
  desiredA = DESIREDA_MIN;
  desiredB = 0;
  desiredY = 0;
  desiredClawRot = 0;
  claw = 1;
  moveDir = 0;
}

//================ read sensor signal ===============

/* Read accelerometer and flex sensor data. */
void readSensors(){
  lsm.read();

  // low pass filter coefficients
  double a = 0.1;
  double b = 0.5;

  // convert accelerometer reading to gravity unit
  accel[0] = (lsm.accelData.x - ACCEL_BIAS[0]) / ACCEL_SENSITIVITY[0];
  accel[1] = (lsm.accelData.y - ACCEL_BIAS[1]) / ACCEL_SENSITIVITY[1];
  accel[2] = (lsm.accelData.z - ACCEL_BIAS[2]) / ACCEL_SENSITIVITY[2];

  // retrieve flex sensor voltage reading
  flex[0] = map(analogRead(FLEX_SENSOR_PIN[0]), 450, 560, 0, 10);
  flex[1] = map(analogRead(FLEX_SENSOR_PIN[1]), 200, 500, 0, 10);
  flex[2] = map(analogRead(FLEX_SENSOR_PIN[2]), 350, 510, 0, 10);

  // low pass filter
  for (int i = 0; i < 3; i++) {
    accel[i] = (1 - a) * prevAccel[i] + a * accel[i];
    prevAccel[i] = accel[i];
    flex[i] = (1 - b) * prevFlex[i] + b * flex[i];
    prevFlex[i] = flex[i];
  }

  // calculate pitch and roll
  roll = atan2(accel[1], -accel[2]) * 180 / pi;
  pitch = atan2(-accel[0], sqrt(pow(accel[1],2) + pow(accel[2],2))) * 180 / pi;
}

//================ calculate coordinates ============

/* Calculate desired coordinates based on sensor signal. */
void calculatePosition() {
  // set arm movement mode based on flex0 reading
  // 0: control AB axes, 1: control Y axes, 2: control vehicle
  int mode = (flex[0] <= 3) ? 1 : 0;

  // switch to vehicle mode if flex1 is bent to certain degree
  if (flex[1] <= 1) {
    mode = 2;
  } else {
    moveDir = 0;
  }

  // close claw if flex1 is bent to certain degree
  claw = (flex[1] <= 5) ? 1 : 0;

  // rotate claw if flex2 is bent or unbent to certain degree
  if (flex[2] <= 2) {
    desiredClawRot++;
  } else if (flex[2] >= 8) {
    desiredClawRot--;
  }

  // when hand is tilted left/right
  if (abs(roll) > 5) {
    if (mode == 0) {
      desiredB = desiredB + roll * 0.005;
    }
  }
  
  // when hand is tilted up/down
  if (abs(pitch) > 5) {
    if (mode == 0) {
      desiredA = desiredA - pitch * 0.005;
    } else if (mode == 1) {
      desiredY = desiredY + pitch * 0.005;
    }
  }

  // vehicle control
  if (mode == 2) {
    if (abs(roll) > 25) {
      moveDir = (roll > 0) ? 4 : 3;
    } else if (abs(pitch) > 10) {
      moveDir = (pitch > 0) ? 2 : 1;
    } else {
      moveDir = 0;
    }
  }

  // safety
  desiredA = (desiredA > DESIREDA_MAX) ? DESIREDA_MAX : desiredA;
  desiredA = (desiredA < DESIREDA_MIN) ? DESIREDA_MIN : desiredA;
  desiredB = (desiredB > DESIREDB_MAX) ? DESIREDB_MAX : desiredB;
  desiredB = (desiredB < DESIREDB_MIN) ? DESIREDB_MIN : desiredB;
  desiredY = (desiredY > DESIREDY_MAX) ? DESIREDY_MAX : desiredY;
  desiredY = (desiredY < DESIREDY_MIN) ? DESIREDY_MIN : desiredY;
  desiredClawRot = (desiredClawRot > DESIREDCLAWROT_MAX) ? DESIREDCLAWROT_MAX : desiredClawRot;
  desiredClawRot = (desiredClawRot < DESIREDCLAWROT_MIN) ? DESIREDCLAWROT_MIN : desiredClawRot;

  // convert (A, B) Cartesian coordinate to (X, rot) polar coordinate
  desiredX = sqrt(pow(desiredA, 2) + pow(desiredB, 2)) - ARM_12_LENGTH;
  desiredRot = atan2(desiredA, desiredB) * 180 / pi;
}

//================ pack contoller signal ============

/* Pack signal to be sent.
 * Signal structure:
 *    [0]: sign of desiredX coordinate
 *    [1]: value of desiredX * 10
 *    [2]: sign of desiredY coordinate
 *    [3]: value of desiredY * 10
 *    [4]: value of desiredRot (base angle)
 *    [5]: value of desiredClawRot (claw angle)
 *    [6]: claw open/shut
 *    [7]: vehicle movement direction
 */
void pack() {
  payload[0] = (desiredX >= 0) ? 0 : 1;
  payload[1] = abs(int(desiredX * 10));
  payload[2] = (desiredY >= 0) ? 0 : 1;
  payload[3] = abs(int(desiredY * 10));
  payload[4] = int(desiredRot);
  payload[5] = int(desiredClawRot);
  payload[6] = claw;
  payload[7] = moveDir;
}

//================ verify remote response ===========

/* Verify robotic arm has received controller signal. */
void verifyResponse() {
  if (xbee.readPacket(5000)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // turn on LED if success
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

//================ automated demo ===================

/* Perform pre-planned path.
 * (not used in normal mode)
 */
void demo1() {
  t++;
  if (t < 100) {
    claw = 0;
    SimMoveTo(18, -10, 1, 0, 100 - t);
  } else if (t < 200) {
    SimMoveTo(24, -10, 1, 0, 200 - t);
  } else if (t < 300) {
    claw = 1;
    SimMoveTo(24, -10, 10, 0, 300 - t);
  } else if (t < 400) {
    SimMoveTo(18, 3, 10, 0, 400 - t);
  } else if (t < 600) {
    SimMoveTo(18, 7, 0, 180, 600 - t);
  } else if (t < 700) {
    SimMoveTo(26, -10, 6, 0, 700 - t);
  } else if (t < 800) {
    claw = 0;
    SimMoveTo(18, -10, 4, 0, 800 - t);
  } else if (t < 900) {
    SimMoveTo(18,0, 0, 0, 900 - t);
  }
  desiredX = sqrt(pow(desiredA, 2) + pow(desiredB, 2)) - ARM_12_LENGTH;
  desiredRot = atan2(desiredA, desiredB) * 180 / pi;
}

//================ demo helper function =============

/* Go to planned location with given speed.
 * (not used in normal mode)
 */
void SimMoveTo(double goalA, double goalB, double goalY, double goalClawRot, int goalCycles){
  desiredA = desiredA + (goalA - desiredA) / goalCycles;
  desiredB = desiredB + (goalB - desiredB) / goalCycles;
  desiredY = desiredY + (goalY - desiredY) / goalCycles;
  desiredClawRot = desiredClawRot + (goalClawRot - desiredClawRot) / goalCycles;
}

