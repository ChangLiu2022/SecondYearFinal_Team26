from machine import Pin, ADC, PWM

vret_pin = ADC(Pin(26)) #vret means voltage return and is the voltage on the current sensor resistor side, real vout is vout-vret
vout_pin = ADC(Pin(28)) #the numbers on the pins are the numbers next to GP (general purpose)
vin_pin = ADC(Pin(27))
pwm = PWM(Pin(0))
pwm.freq(100000)
pwm_en = Pin(1, Pin.OUT)

count = 0
pwm_out = 0
pwm_ref = out = 25000
#Ts--> 1 kHz control frequency. It's better to design the control period as integral multiple of switching period.
#e->error; 0->this time; 1->last time; 2->last last time
ui_max=62300 #anti-windup limitation
ui_min=120 #anti-windup limitation
Ts=0.001
kpi=30
kii=50
kdi=0
out1 = 0
e1 = 0
e2 = 0
CL_mode = 1
current_aim = 0.1
current_limit = 0.3

def saturate(duty):
    if duty > 62500:
        duty = 62500 #max duty cycle is set to 95.3% here
    if duty < 100:
        duty = 100 #min duty cycle is set to 0.15% here
    return duty

def saturation(sat_input, uplim, lowlim): # Saturation function 
    if sat_input > uplim:
        sat_input = uplim
    elif sat_input < lowlim: 
        sat_input = lowlim
    return sat_input

while True:
    
    pwm_en.value(1)

    vin = ((12490/2490)*3.3)*(vin_pin.read_u16())/(2**16 - 1) #making digital 16-bit measurements into voltage
    vout = ((12490/2490)*3.3)*(vout_pin.read_u16())/(2**16 - 1) #making digital 16-bit measurements into voltage and calculating the voltage vout from the voltage at the pins
    vret = 3.3*(vret_pin.read_u16())/(2**16 - 1) #making digital 16-bit measurements into voltage
    count = count + 1
  
    actual_vout = vout - vret
    current_ret = vret/1.02 #1.02 is the current sensor resistor value
    actual_current = current_ret
    
    # Closed Loop Buck
    # The current loop gives a duty cycle demand based upon the error between demanded current and measured current
    # We can/should apply only current pid closed loop and not both voltage pid because the LED works with current and the voltage pid would work well with resistors not LEDS
    if CL_mode == 1: #close loop mode
        current_aim = saturation(current_aim, current_limit, 0)
        e = current_aim - actual_current
        e_integration = e
        if (out1 >= ui_max) or (out1 <= ui_min): #anti-windup, if last-time pid output reaches the limitation, this time there won't be any intergrations.
            e_integration = 0
        dout = kpi * e + kii * Ts * e_integration + kdi / Ts * (e - 2 * e1 + e2)  #incremental PID programming avoids integrations.there is another PID program called positional PID.
        out = out1 + dout #this time's control output
        out = saturation(out, 62500, 100)
        out1 = out # update last time's control output
        e2 = e1  #update last last time's error 
        e1 = e  # update last time's error
        pwm_ref = out
    else: #open loop mode
        oc = actual_current - current_limit
        if oc>0:
            pwm_ref = pwm_ref-0.001
        else:
            pwm_ref = pwm_ref+0.001
            
    pwm_out = saturate(pwm_ref)
    pwm.duty_u16(int(pwm_out))
    pwm_out = pwm_out/(2**16 - 1) #making duty cycle from 0 to 65535 to 0 to 1

    if count > 2000:
        print("Vin = {:.0f}".format(vin)) #This line prints the value of the variable vin to the console. The value is formatted with no decimal places using the "{:.0f}" format specifier.
        print("Vout = {:.0f}".format(vout))
        print("Vret = {:.0f}".format(vret))
        print("Actual_vout = {:.0f}".format(actual_vout))
        print("Actual_current = {:}".format(actual_current))
        print("Duty = {:}".format(pwm_out))
        
        count = 0
