/*
 * Based upon code written by Yue Zhu (yue.zhu18@imperial.ac.uk) in July 2020.
 * pin6 is PWM output at 62.5kHz.
 * duty-cycle saturation is set as 2% - 98%
 * Control frequency is set as 1kHz. 
*/
#include <Wire.h>
#include <INA219_WE.h>

INA219_WE ina219; // this is the instantiation of the library for the current sensor

float Vo,Vi,vref,iL,dutyref,current_mA; // Measurement Voriables
unsigned int sensorVolue0,sensorVolue1,sensorVolue2,sensorVolue3;  // ADC sample Volues declaration
float ev=0,cv=0,ei=0,oc=0; //internal signals
float Ts=0.001; //1 kHz control frequency. It's better to design the control period as integral multiple of switching period.
float Kp, Ki, Kd;
float aim, dout, out, out1, e, e1, e2, out_max, out_min;
float uv_max=4, uv_min=0; //anti-windup limitation
float ui_max=50, ui_min=0; //anti-windup limitation
float current_limit = 1.0;
float P; //Power
float P_previous = 0; //Previous power for comparison in the MPPT algorithm
boolean Boost_mode = 1;
boolean CL_mode = 0;
boolean mppt = 1;
unsigned int loopTrigger;
unsigned int com_count=0;   // a Voriables to count the interrupts. Used for program debugging.
float duty_cycle = 0.5;
float dutyStep = 0.008;
int count = 0;

void setup() {

  //Basic pin setups
  
  noInterrupts(); //disable all interrupts
  pinMode(13, OUTPUT);  //Pin13 is used to time the loops of the controller
  pinMode(3, INPUT_PULLUP); //Pin3 is the input from the Buck/Boost switch
  pinMode(2, INPUT_PULLUP); // Pin 2 is the input from the CL/OL switch
  analogReference(EXTERNAL); // We are using an external analogue reference for the ADC

  // TimerA0 initialization for control-loop interrupt.
  
  TCA0.SINGLE.PER = 999; //
  TCA0.SINGLE.CMP1 = 999; //
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; //16 prescaler, 1M.
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP1_bm; 

  // TimerB0 initialization for PWM output
  
  pinMode(6, OUTPUT);
  TCB0.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm; //62.5kHz
  analogWrite(6,120); 

  Serial.begin(115200);   //serial communication enable. Used for program debugging.
  interrupts();  //enable interrupts.
  Wire.begin(); // We need this for the i2c comms for the current sensor
  ina219.init(); // this initiates the current sensor
  Wire.setClock(700000); // set the comms speed for i2c
  
}

void loop() {
  if(loopTrigger) { // This loop is triggered, it wont run unless there is an interrupt
    
    digitalWrite(13, HIGH);   // set pin 13. Pin13 shows the time consumed by each control cycle. It's used for debugging.
    
    // Sample all of the measurements and check which control mode we are in
    sampling();
    CL_mode = digitalRead(3); //input from the OL_CL switch
    Boost_mode = digitalRead(2); //input from the Buck_Boost switch

  if (Boost_mode){
    if (Vo < 17){
      if (CL_mode) { //Closed Loop Boost
        pwm_modulate(1); // This disables the Boost as we are not using this mode
      }
      else if (mppt) { //MPPT control
        duty_cycle = MPPT(Vi, iL, P_previous);
        duty_cycle = saturate(duty_cycle, 0.99, 0.01); //Maximum duty cycle: 0.99, minimum duty cycle: 0.01
        pwm_modulate(duty_cycle); // and send it out
      }
      else { // Open Loop Boost
        current_limit = 2;
        oc = iL - current_limit; // Calculate the difference between current measurement and current limit
        if (oc > 0) {
        duty_cycle += 0.001; // We are above the current limit so less duty cycle
        }
        else {
        duty_cycle -= 0.001; // We are below the current limit so more duty cycle
        }
      }
    }
    else {
      pwm_modulate(1); // This disables the Boost to avoid overvoltage and burning of the led driver/orBOOST 
    }
  }
  else{      
      if (CL_mode) { // Closed Loop Buck
          pwm_modulate(1); // This disables the Buck as we are not using this mode
      }else{ // Open Loop Buck
          pwm_modulate(1); // This disables the Buck as we are not using this mode
      }
  }
    digitalWrite(13, LOW);   // reset pin13.
    loopTrigger = 0;
  }
}


// Timer A CMP1 interrupt. Every 800us the program enters this interrupt. 
// This, clears the incoming interrupt flag and triggers the main loop.

ISR(TCA0_CMP1_vect){
  TCA0.SINGLE.INTFLAGS |= TCA_SINGLE_CMP1_bm; //clear interrupt flag
  loopTrigger = 1;
}

// This subroutine processes all of the analogue samples, creating the required Volues for the main loop

void sampling() {
  // Make the initial sampling operations for the circuit measurements
  sensorVolue0 = analogRead(A0); //sample Vi
  sensorVolue2 = analogRead(A2); //sample Vref
  sensorVolue3 = analogRead(A3); //sample Vo
  current_mA = ina219.getCurrent_mA(); // sample the inductor current (via the sensor chip)

  // Process the Volues so they are a bit more usable/readable
  // The analogRead process gives a Volue between 0 and 1023 
  // representing a voltage between 0 and the analogue reference which is 4.096V
  
  Vi = sensorVolue0 * (12400/2400) * (4.096 / 1023.0); // Convert the Vi sensor reading to volts
  vref = sensorVolue2 * (4.096 / 1023.0); // Convert the Vref sensor reading to volts
  Vo = sensorVolue3 * (12400/2400) * (4.096 / 1023.0); // Convert the Vo sensor reading to volts

  // The inductor current is in mA from the sensor so we need to convert to amps.
  // We want to treat it as an input current in the Boost, so its also inverted
  // For open loop control the duty cycle reference is calculated from the sensor
  // differently from the Vref, this time scaled between zero and 1.
  // The boost duty cycle needs to be saturated with a 0.33 minimum to prevent high output voltages
  
  if (Boost_mode == 1){
    iL = -current_mA/1000.0;
    dutyref = saturate(sensorVolue2 * (1.0 / 1023.0),0.99,0.33);
  }else{
    iL = current_mA/1000.0;
    dutyref = sensorVolue2 * (1.0 / 1023.0);
  }
}

float saturate( float sat_input, float uplim, float lowlim){ // Saturation function
  if (sat_input > uplim) sat_input=uplim;
  else if (sat_input < lowlim ) sat_input=lowlim;
  else;
  return sat_input;
}

void pwm_modulate(float pwm_input){ // PWM function
  analogWrite(6,(int)(255-pwm_input*255)); 
}

float MPPT(float Vi, float iL, float& P_previous) {
  P = Vi * iL;
  if( count == 0 ){
     delay(1000);
     count = 1;
  }
  delay(500);
  // Check if power has increased or decreased
        if (P_previous < P) {
          // Power has increased, move in the same direction
          duty_cycle += dutyStep;
          // Repeat the last change if power has increased
          dutyStep = dutyStep;
        } else {
          // Power has decreased, move in the opposite direction
          duty_cycle -= dutyStep;
          // Store the current step as the opposite direction for future reference
          dutyStep = -dutyStep;
        }
  P_previous = P;

  //used for debugging.
  Serial.print("Vo: ");
  Serial.print(Vo);
  Serial.print("\t");
  Serial.print("Vi: ");
  Serial.print(Vi);
  Serial.print("\t");
  Serial.print("Inductor Current: ");
  Serial.print(iL);
  Serial.print("\t\t");
  Serial.print("duty_cycle: ");
  Serial.print(duty_cycle, 4);
  Serial.print(" ");
  Serial.print("Power: ");
  Serial.print(P, 4);
  Serial.print(" ");
  Serial.print("Previous Power: ");
  Serial.println(P_previous, 4);
    
  return duty_cycle;
}
