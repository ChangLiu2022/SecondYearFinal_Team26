#include <string>
//Control plant specification:
//Unstable
//180 steps per second or 180 x 16 microsteps per second(1 revolution, speed);
//200 steps or 200 * 16 microsteps(1 revolution, position);
//Wheel radius 3.35 cm, Axial length from wheel centre 14.3 cm
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
#define pi2 6.283185307

//Motor setup
AccelStepper motor1(1, stp1, dir1);
AccelStepper motor2(1, stp2, dir2);

float w, absw, rps, sps1, sps2, W;
//Sensor setup
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float vl = 0;
float xl = 0;
float theta_rad = 0;
float ewy_sum = 0, eax_sum = 0, eay_sum = 0, eaz_sum = 0, sdax = 0, sday = 0, sdaz = 0, sdwy = 0;
float ewy, eax, eay, eaz, t, t1, dt, dt1, kalmangain, trig, theta;
float kalmanunc = sq(2);
int leftl;

//Control system setup
float aim, in, e, e1, e2, P1_out, PID1_out, dPID1_out, PID2_out, dPID2_out, angle, t2, t3, t4, t5, dt2, dt3, offset, V, distance, de, de1, de2, velocity, alpha;
float new_dis = 0, Theta = 0;
float e11 = 0;
float e21 = 0;
int count = 0;
int mode, turn_dir, move_dir;
std::string MOVE_DIR, TURN_DIR;

#define pw 0.2412743158
#define pl 0.4492477495

//ESP32 duo core operation setup
TaskHandle_t Task1;
TaskHandle_t Task2;
//====================================================================MAIN===============================================================================

void setup() {
  Serial.begin(115200);
  pinMode(MS11, OUTPUT); // Set microstepping pins for the motor driver
  pinMode(MS12, OUTPUT);
  pinMode(MS13, OUTPUT);
  pinMode(MS21, OUTPUT);
  pinMode(MS22, OUTPUT);
  pinMode(MS23, OUTPUT);
  digitalWrite(MS11, HIGH); // Set all microstepping pins to high for 16th step
  digitalWrite(MS12, HIGH);
  digitalWrite(MS13, HIGH);
  digitalWrite(MS21, HIGH);
  digitalWrite(MS22, HIGH);
  digitalWrite(MS23, HIGH);
  sensor_setup();
  motor_setup();
  t1 = micros() / 1e6; //The first time we read time before the sensor starts to fetch the data
  t3 = micros() / 1e6;
  t5 = micros() / 1e6;

  //ESP32 duo core setup, create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Sensing",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Control",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
}

//Task1code: sensing
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    sensor_exe();
  } 
}

//Task2code: control
void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    motor_exe();
  }
}

void loop() {
  // This loop is not used here.
}

void motor_setup() {
  motor1.setMaxSpeed(540 * 16); // Max. speed = 3 revolutions per second
  motor2.setMaxSpeed(540 * 16);
}

void motor_exe() {
  precalibrate();
  if (mode == 0) {
    move();
  }
  else if (mode == 1) {
    turn();
  }
  balance();
}

void sensor_setup() {
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Use low pass flter to reduce noise interference
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
  //calib();
}

void calib() {
  Serial.print("Begin calibration.");
  delay(1000);
  int i;
  for (i = 0; i < 10000; i++) { // Take 10000 measurements
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.print("Measured "); Serial.print(i); Serial.println(" setpoints.");
    ewy_sum += g.gyro.y; // Add the stationary error overtime
  };
  ewy = ewy_sum / 10000; // Divide by the no. of samples to get an average
  Serial.println("Calibration complete.");
  Serial.println(ewy);
  delay(1000);
}

void sensor_exe() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  w = g.gyro.y + 0.01; // Subtract stationary error
  //Kp = (abs(theta) < 9) ? 1 : 20; // Gain scheduling; old idea not used here
  a.acceleration.x = a.acceleration.x - 0.45; // Subtract stationary erro from each
  a.acceleration.y = a.acceleration.y + 0.28;
  a.acceleration.z = a.acceleration.z + 0.6;
  trig = atan(- a.acceleration.x / sqrt((sq(a.acceleration.y) + sq(a.acceleration.z)))); // Trigonometric method to calculate pitch angle from acceleration
  kalman(theta_rad, kalmanunc, w, trig);
  /*Serial.print(velocity, 6);
  Serial.print(" ");
  Serial.print(distance, 6);
  Serial.print(" ");*/
  Serial.println(theta);
}

void kalman(float& theta_rad, float& kalmanunc, float& w, float& trig) { // Complementary filter called "Kalman filter"
  t = micros() / 1e6; // Get second time
  dt = t - t1; // Calculate time elapsed
  theta_rad += dt * w; // Discrete integraion for pitch angle
  kalmanunc += sq(dt) * sq(4); // Integrate Kalman uncertainty overtime
  t1 = micros() / 1e6; // Get first time
  kalmangain = kalmanunc / (kalmanunc + sq(3)); // Calculate Kalman gain
  theta_rad += kalmangain * (trig - theta_rad); // Adjust pitch angle integral based on Kalman gain
  theta = theta_rad * 360 / pi2; // Convert from radian to degree
  kalmanunc *= (1 - kalmangain); // Update Kalman uncertainty every iteration
}

//=================================================================CONTROL=======================================================================================
float saturate( float sat_input, float ub, float lb){
  sat_input = min(max(sat_input, lb), ub); 
  return sat_input;
}

float P(float in, float aim, float Kp, float max, float min) { // 
  e = aim - in;
  P1_out = Kp * e;
  P1_out = saturate(P1_out, max, min); // Saturate output depending on the operation condition
  //Serial.println(e, 6);
  return P1_out;
}

float PI1(float in, float aim, float Kp1, float Ki1, float Kd1, float max1, float min1) { // PID controller for pitch angle velocity loop
  e1 = aim - in;
  t2 = micros() / 1e6; // Get first time
  dt2 = t2 - t3; // Calculate time elpased
  dPID1_out += Ki1 * e1 * dt2; // Discrete integration for error
  dPID1_out = saturate(dPID1_out, max1, min1); // Saturate error integral depending on the operation condition
  t3 = micros() / 1e6; // Get first time
  PID1_out = Kp1 * e1 + dPID1_out + Kd1 * (e1 - e11) / dt2; // Final output
  //Serial.println(dPID1_out, 6);
  e11 = e1; // Update error
  return PID1_out;
}

float PI2(float in, float aim, float Kp2, float Ki2, float Kd2, float max2, float min2) { // PID controller for velocity loop
  e2 = aim - in;
  t4 = micros() / 1e6; // Get second time
  dt3 = t4 - t5; // Calculate time elapsed
  dPID2_out += Ki2 * e2 * dt3; // Discrete integration for error
  dPID2_out = saturate(dPID2_out, max2, min2); // Saturate error integral depending on the operation condition
  dPID2_out = (e21 < 1e-3) ? 0 : dPID2_out; // Anti-windup prevents distance overshoot 1e-6 for constant velocity
  t5 = micros() / 1e6; // Get first time
  PID2_out = Kp2 * e2 + dPID2_out + Kd2 * (e2 - e21) / dt3; // Final output
  //Serial.println(dPID2_out, 6);
  e21 = e2; // Update the error
  return PID2_out;
}

void precalibrate() { // Precalibration function estab;ish the 0 position of the robot at the start
  if (count == 50001) {
    de = motor1.currentPosition() - motor2.currentPosition(); // Get the distance errors at the start
    de1 = motor1.currentPosition(); 
    de2 = motor2.currentPosition(); 
  }
  if (count >100000) {
    distance = pw * 26 / 30 * (motor1.currentPosition() - motor2.currentPosition() - de) / (2 * 3200); // Calibrate the distance as it goes, using dead-reckoning
    velocity = pw * (motor1.speed() - motor2.speed()) / (2 * 3200); // Velocity measurements are accurate enough to go without calibration, using dead_reckoning
    alpha = 710 * pw * (motor1.currentPosition() - de1 + motor2.currentPosition() - de2) / (2 * 3200); // Dead reckoning for turning angle
  }
}

void balance() { // CHECK EQUILIBRIUM POINT FIRST!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  W = P(theta, 0 - Theta, 100, 1e6, -1e6); // Duo loop control strategy 100; 340; 100; -4.5 with cable
  sps1 = PI1(w * 360 / pi2, W, 14, 140, 0, 360 * 16, -360 * 16); // 7; 2.2, 0.1, ?; 14, 140, ?
  motor1.setSpeed(sps1 + offset); // Apply a veloity difference between 2 wheels
  motor2.setSpeed(-sps1 + offset);
  motor1.runSpeed();
  motor2.runSpeed();
}

void move() {
  move_dir = (MOVE_DIR == "FORWARD") ? 1 : -1; // Move forward or backward
  V = P(distance, new_dis * move_dir, 5, 0.25, -0.25); // Duo loop controller for distance and velocity (0.3, 4.5), (0.2, )
  Theta = PI2(velocity, 0.3, 20, 40, 0, 2.5, -2.5); // For constant velocity: 20, 40, 0, 2.5, -2.5
}

void turn() {
  turn_dir = (TURN_DIR == "RIGHT") ? 1 : 0; // Turn left or right
  offset = (count > 100000) ? P(alpha, 2000, 1000 * turn_dir, 900, -900) : 0; // Schmit trigger controller
}