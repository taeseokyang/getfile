import serial
import adafruit_bno055
import time
import RPi.GPIO as GPIO
uart = serial.Serial('/dev/ttyAMA2')
sensor = adafruit_bno055.BNO055_UART(uart)

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

wing_pins = [14,15,18,21]

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
			angle = 130
		elif ua > 5:
			angle = 120
		else:
			angle = 110
	elif amount > 0:
		way = 'right'
		ua = abs(amount)
		if ua > 15:
			angle = 50
		elif ua > 5:
			angle = 60
		else:
			angle = 70
	else:
		way = 'stop'
		angle = 90
	
	print(way,amount,now)
	for i in range(4):
		setServoPos(angle, servos[i])


	before = now
import time

pid_x = PID(kp=1.0, ki=0.1, kd=0.05)
pid_y = PID(kp=1.0, ki=0.1, kd=0.05)
pid_z = PID(kp=1.0, ki=0.1, kd=0.05)

# 날개의 기본 각도는 90도
wing_angle = [90, 90, 90, 90]

# 센서 데이터를 읽고 PID 제어기를 사용하여 날개의 각도를 조정하는 함수
def adjust_wings_to_stabilize(current_x, current_y, current_z):
    global wing_angle

    # 각 PID 제어기로부터 보정값을 계산
    correction_x = pid_x.update(current_x)
    correction_y = pid_y.update(current_y)
    correction_z = pid_z.update(current_z)

    # 날개의 각도를 조정 (단순화된 예제, 실제로는 더 복잡한 계산이 필요할 수 있음)
    wing_angle[0] = 90 + correction_x - correction_y + correction_z
    wing_angle[1] = 90 - correction_x - correction_y - correction_z
    wing_angle[2] = 90 + correction_x + correction_y - correction_z
    wing_angle[3] = 90 - correction_x + correction_y + correction_z

    # 날개의 각도는 0도에서 180도 사이로 제한
    wing_angle = [max(0, min(180, angle)) for angle in wing_angle]

    return wing_angle

while True:
    x_angle = 100  # 센서로부터 x 각도 읽기
    y_angle = 80   # 센서로부터 y 각도 읽기
    z_angle = -80  # 센서로부터 z 각도 읽기

    wing_angles = adjust_wings_to_stabilize(x_angle, y_angle, z_angle)
    print(f"Wing Angles: {wing_angles}")

    time.sleep(0.1)
