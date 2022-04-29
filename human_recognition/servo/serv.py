import Jetson.GPIO as GPIO
import time

#8 is stop
#4 is 45 degr
#2.5 is 90 degr

control = [2.5,4.5,8]
#control = [5, 7.5, 10]
servo = 32 #GPIO pin

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo, GPIO.OUT)
# in servo motor,
# 1ms pulse for 0 degree (LEFT)
# 1.5ms pulse for 90 degree (MIDDLE)
# 2ms pulse for 180 degree (RIGHT)

# so for 50hz, one frequency is 20ms
# duty cycle for 0 degree = (1/20)*100 = 5%
# duty cycle for 90 degree = (1.5/20)*100 = 7.5%
# duty cycle for 180 degree = (2/20)*100 = 10%

p=GPIO.PWM(servo,50) # 50hz frequency
p.start(2.5) # starting duty cycle ( it set the servo to 0 degree )

try:
        while True:
                for x in range(len(control)):
                        p.ChangeDutyCycle(control[x])
                        time.sleep(1)
                        print(control[x])

                for x in range(len(control) - 2, 0, -1):
                        p.ChangeDutyCycle(control[x])
                        time.sleep(1)
                        print(control[x])

except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()