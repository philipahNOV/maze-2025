import Jetson.GPIO as GPIO
import time
import random
 
# Pin Definitions
pwmOutPin = 33  # PWM output pin (Jetson Nano pin 32)

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(pwmOutPin, GPIO.OUT)
 
# Setup PWM on output pin
pwm = GPIO.PWM(pwmOutPin, 1000)  # Initialize PWM at 1kHz frequency
pwm.start(50)  # Start PWM with 50% duty cycle
counter = 0
flipp = True
try:
    while True:
        if counter % 1000 == 0:
            print("Counter: ", counter)
            if flipp:
                pwm.ChangeDutyCycle(100)
                flipp = False
            else:
                pwm.ChangeDutyCycle(0)
                flipp = True
        # PWM is running, nothing else to do
        time.sleep(0.2)  # Sleep to reduce CPU usage
 
except KeyboardInterrupt:
    pass
 
finally:
    pwm.stop()
    GPIO.cleanup()