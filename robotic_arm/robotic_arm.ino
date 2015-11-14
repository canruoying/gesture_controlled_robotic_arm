#include <Servo.h>
#include <XBee.h>

//================ servo constants ==================

//servo pins
const int SERVO_PIN[6] = {8, 9, 10, 11, 12, 13}; //replaced 8-13
//control signal max and mins
const int SERVO_MIN[6] = {720, 480, 480, 630, 720, 750};
const int SERVO_MAX[6] = {2330, 2520, 2520, 2300, 2330, 2330};
//degrees of freedom safety restrictions
const int SERVO_ANGLE_MAX[6] = {180, 180, 180, 180, 180, 110};
const int SERVO_ANGLE_MIN[6] = {0, 0, 30, 0, 0, 60};

//================ motor constants ==================
//motor pins
const int LEFT_MOTOR_POS = 4;
const int LEFT_MOTOR_NEG = 5;
const int RIGHT_MOTOR_POS = 6;
const int RIGHT_MOTOR_NEG = 7;

//================ calculation constants ============

//armature lengths for servos 9 and 10
const double ARM_9_LENGTH = 10.4;
const double ARM_10_LENGTH = 9.8;
//pi
const double pi = 3.14;
//reach
const int DESIREDY_MIN = -10;
const int DESIREDY_MAX = 12;
const int DESIREDX_MIN = 4;
const int DESIREDX_MAX = 16;
const int DESIREDROT_MAX = 180;
const int DESIREDROT_MIN = 0;

//================ servo/motor variables ============

//servo
Servo SERVO[6];
//current angles and desired angles
volatile int servo_angle[6]= {90, 30, 30, 180, 0, 60};
volatile int servo_angle_desired[6] = {90, 30, 30, 180, 0, 60};
//desired coordinates
volatile double desiredX;
volatile double desiredY;
volatile int desiredRot;    // 0: right, 180: left
volatile int claw;          // 0: open, 1:closed
volatile int moveDir;           // 0: stop, 1: forward, 2: reverse, 3: left, 4: right

//================ XBee variables ===================

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

//================ simulation variables =============
int t = 0;

void setup() {
  Serial.begin(9600);
  xbee.setSerial(Serial);

  //initialize servo pins
  for (int i = 0; i < 6; i++) {
    SERVO[i].attach(SERVO_PIN[i], SERVO_MIN[i], SERVO_MAX[i]);
  }

  //initialize motor pins
  pinMode(LEFT_MOTOR_POS, OUTPUT);
  pinMode(LEFT_MOTOR_NEG, OUTPUT);
  pinMode(RIGHT_MOTOR_POS, OUTPUT);
  pinMode(RIGHT_MOTOR_NEG, OUTPUT);
  
  //reset position
  positionReset();
  controlServo();
  delay(2000);
}

void loop() {
  //get control signal from XBee
  getSignal();

  //calculate desired servo angles based on desired coordinates
  calculateAngles();

  //change current servo angles to desired angles
  positionChange();

  //write to servos
  controlServo();

  //write to motors
  movementChange();
}

void positionReset() {
  desiredX = 1;
  desiredY = 4;
  desiredRot = 90;
  claw = 0;
  moveDir = 0;
}

void positionChange() {
  for (int i = 0; i < 6; i++) {
    if (servo_angle[i] != servo_angle_desired[i]) {
      servo_angle[i] = servo_angle_desired[i]; //(servo_angle[i] > servo_angle_desired[i]) ? servo_angle[i] - 1 : servo_angle[i] + 1;
    }
    servo_angle[i] = (servo_angle[i] > SERVO_ANGLE_MAX[i]) ? SERVO_ANGLE_MAX[i] : servo_angle[i];
    servo_angle[i] = (servo_angle[i] < SERVO_ANGLE_MIN[i]) ? SERVO_ANGLE_MIN[i] : servo_angle[i];
  }
}

void controlServo() {
  for (int i = 0; i < 6; i++) {
    SERVO[i].write(servo_angle[i]);
  }
}

void movementChange() {
  switch (moveDir) {
    case 1:
      controlMotor(LEFT_MOTOR_POS, LEFT_MOTOR_NEG, 1);
      controlMotor(RIGHT_MOTOR_POS, RIGHT_MOTOR_NEG, 1);
      break;
    case 2:
      controlMotor(LEFT_MOTOR_POS, LEFT_MOTOR_NEG, -1);
      controlMotor(RIGHT_MOTOR_POS, RIGHT_MOTOR_NEG, -1);
      break;
    case 3:
      controlMotor(LEFT_MOTOR_POS, LEFT_MOTOR_NEG, -1);
      controlMotor(RIGHT_MOTOR_POS, RIGHT_MOTOR_NEG, 1);
      break;
    case 4:
      controlMotor(LEFT_MOTOR_POS, LEFT_MOTOR_NEG, 1);
      controlMotor(RIGHT_MOTOR_POS, RIGHT_MOTOR_NEG, -1);
      break;
    default:
      controlMotor(LEFT_MOTOR_POS, LEFT_MOTOR_NEG, 0);
      controlMotor(RIGHT_MOTOR_POS, RIGHT_MOTOR_NEG, 0);
      break;
  }
}

void controlMotor(int pos_pin, int neg_pin, int dir) {
  if (dir == 0) {
    digitalWrite(pos_pin, LOW);
    digitalWrite(neg_pin, LOW);
  } else if (dir == 1) {
    digitalWrite(neg_pin, LOW);
    digitalWrite(pos_pin, HIGH);
  } else {
    digitalWrite(pos_pin, LOW);
    digitalWrite(neg_pin, HIGH);
  }
}

void calculateAngles() {
  double distance = sqrt(pow(desiredX, 2) + pow(desiredY, 2));
  double thetaDesired = atan2(desiredY, desiredX) * 180 / pi;
  double thetaA = acos((pow(ARM_9_LENGTH, 2) - pow(ARM_10_LENGTH, 2) + pow(distance, 2)) / (2 * ARM_9_LENGTH * distance)) * 180 / pi;
  double thetaB = acos((pow(ARM_9_LENGTH, 2) + pow(ARM_10_LENGTH, 2) - pow(distance, 2)) / (2 * ARM_9_LENGTH * ARM_10_LENGTH)) * 180 / pi;
  servo_angle_desired[0] = double(desiredRot) / (DESIREDROT_MAX - DESIREDROT_MIN) * (SERVO_ANGLE_MAX[0] - SERVO_ANGLE_MIN[0]) + SERVO_ANGLE_MIN[0];
  servo_angle_desired[1] = 180 - (thetaA + thetaDesired);
  servo_angle_desired[2] = thetaB;
  servo_angle_desired[3] = thetaA + thetaB + thetaDesired;
  servo_angle[5] = (claw == 1) ? SERVO_ANGLE_MAX[5] : SERVO_ANGLE_MIN[5];
}

void getSignal() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    xbee.getResponse().getRx16Response(rx16);
    desiredX = rx16.getData(1) / 10.0;
    desiredX = (rx16.getData(0) == 0) ? desiredX : -desiredX;
    desiredY = rx16.getData(3) / 10.0;
    desiredY = (rx16.getData(2) == 0) ? desiredY : -desiredY;
    desiredRot = rx16.getData(4);
    claw = rx16.getData(5);
    moveDir = rx16.getData(6);
  }
}
