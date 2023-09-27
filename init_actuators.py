import zmq
import ctypes
import time
import threading
from threading import Thread
import RPi.GPIO as GPIO
import Adafruit_PCA9685

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
#  80% done 
# Tracking module not checked


Motor_A_EN    = 4
Motor_B_EN    = 17

Motor_A_Pin1  = 14
Motor_A_Pin2  = 15
Motor_B_Pin1  = 27
Motor_B_Pin2  = 18

Dir_forward   = 0
Dir_backward  = 1

left_forward  = 0
left_backward = 1

right_forward = 0
right_backward= 1

Seeker_head_move = True

Tr = 11
Ec = 8




def setup():
	global pwm_A, pwm_B
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(Tr,GPIO.OUT, initial = GPIO.LOW)
	GPIO.setup(Ec, GPIO.IN)
	GPIO.setup(Motor_A_EN, GPIO.OUT)
	GPIO.setup(Motor_B_EN, GPIO.OUT)
	GPIO.setup(Motor_A_Pin1, GPIO.OUT)	
	GPIO.setup(Motor_A_Pin2, GPIO.OUT)
	GPIO.setup(Motor_B_Pin1, GPIO.OUT)
	GPIO.setup(Motor_B_Pin2, GPIO.OUT)

	motorStop()
	try:
		pwm_A = GPIO.PWM(Motor_A_EN, 1000)
		pwm_B = GPIO.PWM(Motor_B_EN, 1000)
		# Hand does not respond
		Servo_wrist()
		return True
		
	except:
		pass
		return False


        
def destroy():
	motorStop()
	GPIO.cleanup()
	


def motorStop():
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)
	


def Servos_grip():
	pos_max = 220
	pos_min = 320

	for i in range(pos_min, pos_max, -1):
		#time.sleep(0.003)
		pwm.set_pwm(15, 0 , i)
		
	time.sleep(0.5)

	for i in range(pos_max, pos_min):
		#time.sleep(0.003)
		pwm.set_pwm(15,0,i)


def Servos_head():
	pos_max = 255
	pos_min = 320

	for i in range (pos_min,pos_max, -1):
		#time.sleep(0.005)
		pwm.set_pwm(11, 0, i)
	time.sleep(1)
	Servos_grip()
	for i in range (pos_max, pos_min+15):
		pwm.set_pwm(11, 0,i)
		#time.sleep(0.005) 
	time.sleep(0.4)



def Servo_wrist():
	pos_max = 360
	pos_min = 160

	for i in range (pos_max, pos_min, -1):
		#time.sleep(0.002)
		pwm.set_pwm(14, 0, i)
	time.sleep(0.4)
	Servos_head()
	for i in range (pos_min, pos_max):
		pwm.set_pwm(14, 0,i)
		#time.sleep(0.005) 
	time.sleep(0.4)




if __name__ == '__main__':
	port = "4343"
	init_system_addr = "tcp://127.0.0.1:" + port
	Actuators_OK = setup()
	destroy()
	context = zmq.Context()
	socket = context.socket(zmq.REQ)
	time.sleep(1)
	
	socket.connect(init_system_addr)
	if Actuators_OK == True:
		socket.send_string("Actuators - OK")
	else:
		socket.send_string("Actuators - BAD")

	exit()