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
#define pi2 6.283185307

AccelStepper motor1(1, stp1, dir1);
AccelStepper motor2(1, stp2, dir2);
//Sensor setup
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
//Motor varaibles
float w, absw, rps, sps, dir, W;
float count = 0;

bool move;
//Sensor variables
float vl = 0;
float xl = 0;
//float w = 0;
float theta_rad = 0;
float ewy_sum = 0, eax_sum = 0, eay_sum = 0, eaz_sum = 0, sdax = 0, sday = 0, sdaz = 0, sdwy = 0;
float ewy, eax, eay, eaz, t, t1, t2, t3, t4, t5, dt, dt1, dt2, dt3, kalmangain, trig, theta;
float kalmanunc = sq(2);

//PID controller variables
float aim, in, e, e1, e2, out, dout, Kp1, Ki1, Kd1;
float out1 = 0;
float e11 = 0;
float e21 = 0;
//====================================================================MAIN===============================================================================
void setup() {
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
  sensor_setup();
  motor_setup();
  t1 = micros() / 1e6; //The first time we read time before the sensor starts to fetch the data
  t3 = micros() / 1e6;
}

void loop() {
  sensor_exe();
  motor_exe();
}
//====================================================================MOTOR==============================================================================


void motor_setup() {
  motor1.setMaxSpeed(540 * 16);
  motor2.setMaxSpeed(540 * 16);
}

void motor_exe() {
  //W = P(theta, -9, 600); // Duo loop control strategy 20-40x, 30 
  //sps = P(w * 360 / pi2, W, 50); // 21, 0(0.1-2), 0.3(0.0001-0.001)
  //Serial.println(sps);
  //sps = rps * 180;
  /*
  if (w > 0.1) { // Fuzzy logic controller
    sps = -180;
  }
  else if (w < -0.1) {
    sps = 180;
  }
  else {
    sps = 0;
  }
  */
  motor1.setSpeed(180 * 16);
  motor2.setSpeed(-180 * 16);
  motor1.runSpeed();
  motor2.runSpeed();
}
//=====================================================================SENSOR==========================================================================
void sensor_setup() {
  Serial.begin(115200);
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

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
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
  for (i = 0; i < 10000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.print("Measured "); Serial.print(i); Serial.println(" setpoints.");
    ewy_sum += g.gyro.y;
  };
  ewy = ewy_sum / 10000;
  Serial.println("Calibration complete.");
  Serial.println(ewy);
  delay(1000);
}

void sensor_exe() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //w = g.gyro.y + 0.01;
  
  /*t = micros() / 1e6;// The first dt will just be sensor fetching data time
  dt1 = t - t1; //Afterwards, dt = motor execution time + loop delay + sensor fetching data time
  theta = theta + w * dt1 * 360 / pi2; //Discrete integration
  t1 = micros() / 1e6; //Read the start time before the next round*/
  //theta = (abs(theta) + 72 < 1 && absw < 0.021) ? -72 : theta;
  //Kp = (abs(theta) < 9) ? 1 : 20; // Gain scheduling
  //g.gyro.x = g.gyro.x - ewx;
  //g.gyro.z = g.gyro.z - ewz;
  /*a.acceleration.x = a.acceleration.x - 0.4;
  a.acceleration.y = a.acceleration.y;
  a.acceleration.z = a.acceleration.z;
  trig = atan(- a.acceleration.x / sqrt((sq(a.acceleration.y) + sq(a.acceleration.z))));
  kalman(theta_rad, kalmanunc, w, trig);
  theta = (abs(theta) + 1 < 1 && abs(w) < 0.05) ? -1 : theta;
  Serial.print(theta, 6);
  Serial.print(" ");
  /*Serial.print(a.acceleration.x, 6);
  Serial.print(" ");
  Serial.println(a.acceleration.y, 6);
  Serial.print(" ");
  Serial.println(a.acceleration.z, 6);
  Serial.println(" ");*/
}
//=================================================================CONTROL=======================================================================================
float P(float in, float aim, float Kp) {
  e = aim - in;
  out = Kp * e;
  //Serial.println(e, 6);
  return out;
}

float PI1(float in, float aim, float Kp1, float Ki1, float Kd1) {
  e1 = aim - in;
  t2 = micros() / 1e6;
  dt2 = t2 - t3;
  dout += Ki1 * e1 * dt2;
  t3 = micros() / 1e6;
  Serial.println(dout, 6);
  out = Kp1 * e1 + Kd1 * (e1 - e11) / dt2 + dout; //this time's control output
  e11 = e1;
  return out;
}

float PI2(float in, float aim, float Kp2, float Ki2, float Kd2) {
  e2 = aim - in;
  t4 = micros() / 1e6;
  dt3 = t4 - t5;
  dout += Ki2 * e2 * dt3;  //incremental PID programming avoids integrations.there is another PID program called positional PID.
  t5 = micros() / 1e6;
  out = Kp1 * e2 + Kd2 * (e2 - e21) / dt3;
  out += dout;  //this time's control output
  e21 = e2;
  return out;
}

float saturate( float sat_input, float uplim, float lowlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim) sat_input=lowlim;
  else;
  return sat_input;
}

void kalman(float& theta_rad, float& kalmanunc, float& w, float& trig) {
  t = micros() / 1e6;
  dt = t - t1;
  theta_rad += dt * w;
  kalmanunc += sq(dt) * sq(4);
  t1 = micros() / 1e6;
  kalmangain = kalmanunc / (kalmanunc + sq(3));
  theta_rad += kalmangain * (trig - theta_rad);
  theta = theta_rad * 360 / pi2;
  kalmanunc *= (1 - kalmangain);
}