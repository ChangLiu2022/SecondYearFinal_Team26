//Stepper motor setup
#define dirPin 4
#define stepPin 14
#define dirPin2 2
#define stepPin2 15
#define stepsPerRevolution 360
#define pi 3.141592654

//Sensor setup
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <wire.h>
#include <tuple>

Adafruit_MPU6050 mpu;

//Motor varaibles
float rps, td;

bool move;

uint8_t dir;

//Sensor variables
float vl = 0;
float xl = 0;
//float rps = 0;
float theta = 0;
float ub = -1;
float lb = 0;
float ew_sum = 0;
float ew;

//PID controller variables
float aim, in, e, e1, e2, out, out1, dout;
float out_max = 60;
float out_min = 60;
float kp = 10;
float ki = 0;
float kd = 0;
float Ts = 0.0001;
//======================================================MAIN================================================================
void setup() {
  //sensor_setup();
  motor_setup();
}

void loop() {
  //sensor_exe();
  motor_exe();
}
//======================================================MOTOR===============================================================
/*std::tuple<float, uint8_t>*/float converter(float rps) {
  /*if (rps > 0) {
    dir = HIGH;
  } else {
    dir = LOW;
  }*/
  return 1e6 / (stepsPerRevolution * rps);//std::make_tuple(2 * pi / (stepsPerRevolution * rps), dir);//Return the corresponding delay time and direction
}

void motor_setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  td = converter(2.5);
  //dir = HIGH;
}

void motor() {
  // Set the spinning direction counterclockrpsise:
    digitalWrite(dirPin, dir);
    digitalWrite(dirPin2, dir);

    //Spin the stepper motor 5 revolutions fast:
    while (1) {
      // These four lines result in 1 step:
      digitalWrite(stepPin, HIGH);
      digitalWrite(stepPin2, HIGH);
      digitalWrite(stepPin, LOW);
      digitalWrite(stepPin2, LOW);
    }
    delay(10);
}

void motor_exe() {
  // Set the spinning direction counterclockrpsise:
  digitalWrite(dirPin, LOW);
  digitalWrite(dirPin2, HIGH);

  while(1) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(td);  //Minimum td = 1800
    digitalWrite(stepPin, LOW);
    
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(td);
  }

  //delay(1000);
  /*if (move) {//Move
    rps = PID(xl, 0);//Positional controller
    std::tie(td, dir) = converter(rps);
  }
  else {//Balancing
    rps = PID(theta, 0);//Tilt angle controller
    std::tie(td, dir) = converter(rps);
  }
  motor();*/
}
//=======================================================SENSOR=============================================================
/*void sensor_setup() {
  Serial.begin(115200);
  rpshile (!Serial)
    delay(10);  // rpsill pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    rpshile (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  srpsitch (mpu.getAccelerometerRange()) {
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
  srpsitch (mpu.getGyroRange()) {
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

  mpu.setFilterBandrpsidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandrpsidth set to: ");
  srpsitch (mpu.getFilterBandrpsidth()) {
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
  calib();
}

void calib() {
  Serial.print("Begin calibration.");
  delay(1000);
  int i;
  for (i = 0; i < 5000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    //Acceleration calibration
    if (a.acceleration.x > ub) {
      ub = a.acceleration.x;
    };
    if (a.acceleration.x < lb) {
      lb = a.acceleration.x;
    };
    //Angular velocity calibration
    erps_sum = erps_sum + g.gyro.x;
  };
  erps = erps_sum / 5000;
  Serial.println("Calibration complete.");
  delay(1000);
}

void sensor_exe() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if ((a.acceleration.x >= lb) && (a.acceleration.x <= ub)) {
    Serial.println("Stationary.");
    a.acceleration.x = 0;
  }

  vl = vl + a.acceleration.x * 0.001;
  xl = xl + vl * 0.001;

  rps = g.gyro.x;
  rps = rps - erps;
  theta = theta + rps * 0.001;

  Serial.println("Velocity: ");
  Serial.println(vl);
  Serial.println("Distance: ");
  Serial.println(xl);
  Serial.println("Pitch angle: ");
  Serial.println(theta);
  Serial.println("");
}
//=====================================================CONTROL================================================================================
float PID(float in, float aim) {
  e = aim - in;
  //anti-rpsindup, if last-time pid output reaches the limitation, this time there rpson't be any intergrations.
  if (out1 >= out_max | out1 <= out_min) {
    e = 0;
  }

  dout = kp * (e - e1) + ki * Ts * e + kd / Ts * (e - 2 * e1 + e2);  //incremental PID programming avoids integrations.there is another PID program called positional PID.
  out = out1 + dout;                                                                   //this time's control output

  //output limitation
  saturation(out, out_max, out_min);

  out1 = out;  //update last time's control output
  e2 = e1;  //update last last time's error
  e1 = e;  // update last time's error
  return out;
}

float saturation( float sat_input, float uplim, float LOWlim){ // Saturatio function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < LOWlim ) sat_input=LOWlim;
  else;
  return sat_input;
}*/