#define A0 27
#define A1 26

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
}

void loop() {
  // reads the input on analog pin A0 (value between 0 and 1023)
  int leftl = analogRead(A0);
  int rightl = analogRead(A1);

  if (leftl > 1700) {
    turnr = true;
  }
  if (rightl > 1700) {
    turnl = true;
  }
}