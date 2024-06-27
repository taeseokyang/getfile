import serial
import adafruit_bno055
import time
import RPi.GPIO as GPIO
uart = serial.Serial('/dev/ttyAMA2')
sensor = adafruit_bno055.BNO055_UART(uart)

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

wing_pins = [15,21,18,14]

GPIO.setmode(GPIO.BCM)

servos = []
for i in range(4):
	GPIO.setup(wing_pins[i], GPIO.OUT)
	servo = GPIO.PWM(wing_pins[i], 50)
	servos.append(servo)
	servo.start(0)

def setServoPos(degree, servo):
	duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
	servo.ChangeDutyCycle(duty)

for i in range(4):
	setServoPos(90, servos[i])

class PID:
	def __init__(self, Kp, Ki, Kd):
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.setpoint = 0
		self.prev_error = 0
		self.integral = 0

	def update(self, measured_value):
		error = self.setpoint - measured_value
		self.integral += error
		derivative = error - self.prev_error
		output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
		self.prev_error = error
		return output


pid_x = PID(Kp=0.01, Ki=0.01, Kd=0.01)
pid_y = PID(Kp=0.01, Ki=0.01, Kd=0.01)
pid_z = PID(Kp=0.01, Ki=0.01, Kd=0.01)

wing_angle = [90, 90, 90, 90]

def adjust_wings_to_stabilize(current_x, current_y, current_z):
	global wing_angle

	correction_x = pid_x.update(current_x)
	correction_y = pid_y.update(current_y)
	correction_z = pid_z.update(current_z)

	wing_angle[0] = 90 + correction_x - correction_y + correction_z
	wing_angle[1] = 90 - correction_x - correction_y - correction_z
	wing_angle[2] = 90 + correction_x + correction_y - correction_z
	wing_angle[3] = 90 - correction_x + correction_y + correction_z
	print(wing_angle)
	wing_angle = [max(50, min(130, angle)) for angle in wing_angle]
	
	for i in range(4):
		setServoPos(wing_angle[i],servos[i])
	return wing_angle

while True:
	input_value = sensor.euler
	print(input_value)
	x_angle = input_value[1] 
	y_angle = input_value[2]   
	z_angle = input_value[0]  

	wing_angles = adjust_wings_to_stabilize(x_angle, y_angle, z_angle)

	time.sleep(0.1)
