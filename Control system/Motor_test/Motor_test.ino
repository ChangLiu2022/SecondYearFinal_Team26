//180 steps per second motor
//Motor setup
#include <AccelStepper.h>

#define stp1 14
#define dir1 4
#define stp2 15
#define dir2 2
#define MS11 16
#define MS12 17
#define MS13 5
#define MS21 18
#define MS22 19
#define MS23 23
#define pi 3.141592654
#define pw 0.2412743158
#define pl 0.4492477495

float distance, speed;
int dir, dir_move = 1, count = 0;

AccelStepper motor1(1, stp1, dir1);
AccelStepper motor2(1, stp2, dir2);

//====================================================================MAIN===============================================================================
void setup() {
  Serial.begin(2000000);
  pinMode(MS11, OUTPUT);
  pinMode(MS12, OUTPUT);
  pinMode(MS13, OUTPUT);
  pinMode(MS21, OUTPUT);
  pinMode(MS22, OUTPUT);
  pinMode(MS23, OUTPUT);
  digitalWrite(MS11, HIGH);
  digitalWrite(MS12, HIGH);
  digitalWrite(MS13, HIGH);
  digitalWrite(MS21, HIGH);
  digitalWrite(MS22, HIGH);
  digitalWrite(MS23, HIGH);
  motor_setup();
}

void loop() {
  motor_exe();
}
//====================================================================MOTOR==============================================================================


void motor_setup() {
  motor1.setMaxSpeed(540 * 32);
  motor2.setMaxSpeed(540 * 32);
  motor1.moveTo((long)200 * 16);
  motor2.moveTo((long)200 * 16);
}

void motor_exe() {
  move();
}

void move() {
  motor1.setSpeed(15 * 16);
  motor2.setSpeed(-15 * 16);
  motor1.runSpeed();
  motor2.runSpeed();
}