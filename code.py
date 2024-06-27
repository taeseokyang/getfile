import serial
import adafruit_bno055
import time
import RPi.GPIO as GPIO
uart = serial.Serial('/dev/ttyAMA2')
sensor = adafruit_bno055.BNO055_UART(uart)

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

wing_pins = [2,15,18,21]

GPIO.setmode(GPIO.BCM)

servos = []
for i in range(4):
	GPIO.setup(wing_pins[i], GPIO.OUT)
	servo = GPIO.PWM(wing_pins[i], 50)
	servos.append(servo)
	servo.start(0)

def setServoPos(degree, servo):
	if degree > 180:
		degree = 180
	duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
	servo.ChangeDutyCycle(duty)

for i in range(4):
	setServoPos(90, servos[i])




before = sensor.euler[0]
while True:
	
	time.sleep(0.02)

	
	now = sensor.euler[0]

	amount =int(now - before)

	if amount < 0:
		way = 'left'
		ua = abs(amount)
		if ua > 15:
			angle = 140
		elif ua > 5:
			angle = 140
		else:
			angle = 140
	elif amount > 0:
		way = 'right'
		ua = abs(amount)
		if ua > 15:
			angle =	40
		elif ua > 5:
			angle = 40
		else:
			angle = 40
	else:
		way = 'stop'
		angle = 90
	
	print(way,amount,now)
	for i in range(4):
		setServoPos(angle, servos[i])


	before = now
