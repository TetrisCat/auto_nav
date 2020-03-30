import RPi as GPIO    
import time

GPIO.setmode(GPIO.BOARD)

control_pins=[11,12,13,14]

for pin in control_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin,0)

angle= #need calibrate from what angle shoots the top of the image
    
for i in range(angle/1.8):
    setStep(1,0,1,0)
    setStep(0,1,1,0)
    setStep(0,1,0,1)
    setStep(1,0,0,1)

#switch on DC motors
GPIO.setup(pin_sth, GPIO.OUT,initial=GPIO.HIGH)
GPIO.output(pin_sth,0)    
time.delay(5) #let wheels spin for a while

servo_point = 32,35
GPIO.setup(servo_point,GPIO.OUT)
pwm = GPIO.PWM(servo_point,50)
pwm.start(2.5)
pwm.DutyChangeCycle(5)

