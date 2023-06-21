#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define pi2 6.283185307

Adafruit_MPU6050 mpu;

//Variables
float vl = 0;
float xl = 0;
float w = 0;
float alpha = 0;
float ub = -1;
float lb = 0;
float ewx, ewz, ea;
float dax = 0, day = 0, daz = 0, count = 0;
float theta_rad = 0;
float ewy_sum = 0, eax_sum = 0, eay_sum = 0, eaz_sum = 0;
float ewy, eax, eay, eaz, t, t1, dt, dt1, kalmangain, trig, theta;
float kalmanunc = sq(20);

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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
  //Serial.print("Accelerometer range set to: ");
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
  //Serial.print("Gyro range set to: ");
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

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  //Serial.print("Filter bandwidth set to: ");
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
  t1 = micros() / 1e6;
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /*if ((a.acceleration.x >= lb) && (a.acceleration.x <= ub)) {
    //Serial.println("Stationary.");
    a.acceleration.x = 0;
  }

  if (vl <= 0.28 && a.acceleration.x == 0) {
    vl = 0;
  }
  if (vl < 0) {
    Serial.print("Backward");
  }
  if (vl > 0) {
    Serial.print("Forward");
  }

  a.acceleration.x = a.acceleration.x - ea;

  if (abs(a.acceleration.x) < 0.14) {
    a.acceleration.x = 0;
  }

  if (abs(g.gyro.z) < 0.01) {
    g.gyro.z = 0;
  }

  vl = vl + a.acceleration.x * 0.001;
  theta = theta + g.gyro.x * 0.001;
  alpha = alpha + g.gyro.z * 0.001;

  if (abs(vl) <= 0.05 && abs(a.acceleration.x) < 0.14) {
    vl = 0;
  }*/

  a.acceleration.x = a.acceleration.x - 0.45;
  a.acceleration.y = a.acceleration.y + 0.28;
  a.acceleration.z = a.acceleration.z + 0.6;
  trig = atan(- a.acceleration.x / sqrt((sq(a.acceleration.y) + sq(a.acceleration.z))));
  kalman(theta_rad, kalmanunc, w, trig);
  theta = (abs(theta) + 1 < 1 && abs(w) < 0.05) ? -1 : theta;
  Serial.println(theta);

}

void calib() {
  Serial.println("Begin calibration.");
  delay(1000);

  int i;
  for (i = 1; i <= 10000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    //Acceleration calibration
    /*if (a.acceleration.x > ub) {
      ub = a.acceleration.x;
    };
    if (a.acceleration.x < lb) {
      lb = a.acceleration.x;
    };*/
    //Angular velocity calibration
    Serial.print("Measured "); Serial.print(i); Serial.println(" setpoints.");
    Serial.print(" ");
    Serial.println(a.acceleration.x);
    daz += (a.acceleration.z - 9.81);
    dax += a.acceleration.x;
    day += a.acceleration.y;
    //ewz_sum = ewz_sum + g.gyro.z;
  };

  daz /= 10000;
  dax /= 10000;
  day /= 10000;
  Serial.print("Calibration complete.");
  Serial.print(dax);
  Serial.print(day);
  Serial.print(daz);
  delay(1000);
}

void kalman(float& theta_rad, float& kalmanunc, float& w, float& trig) {
  t = micros() / 1e6;
  dt = t - t1;
  theta_rad += dt * w;
  kalmanunc += sq(dt) * sq(20);
  t1 = micros() / 1e6;
  kalmangain = kalmanunc / (kalmanunc + sq(1));
  theta_rad += kalmangain * (trig - theta_rad);
  theta = trig * 360 / pi2;
  kalmanunc *= (1 - kalmangain);
}