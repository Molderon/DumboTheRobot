import zmq
import re
import ctypes
import time
import threading
import RPi.GPIO as GPIO
import Adafruit_PCA9685

from enum import Enum
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)


 
Motor_A_EN    = 4 	# Don't touch!
Motor_B_EN    = 17	# Don't touch!
					# Don't touch!
Motor_A_Pin1  = 14	# Don't touch!
Motor_A_Pin2  = 15	# Don't touch!
Motor_B_Pin1  = 27	# Don't touch!
Motor_B_Pin2  = 18	# Don't touch!
					# Don't touch!
Dir_forward   = 0	# Don't touch!
Dir_backward  = 1	# Don't touch!
					# Don't touch!
left_forward  = 0	# Don't touch!
left_backward = 1	# Don't touch!
					# Don't touch!
right_forward = 0	# Don't touch!
right_backward= 1	# Don't touch!

Script_Opperational = True
Seeker_head_move = False # Sometimes...

Tr = 11  # Don't touch!
Ec = 8   # Don't touch!




#  Please dobt Judge me, that's my first time writing in Python



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
	except:
		pass



def motor_left(status, direction, speed):#Motor 2 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_B_Pin1, GPIO.LOW)
		GPIO.output(Motor_B_Pin2, GPIO.LOW)
		GPIO.output(Motor_B_EN, GPIO.LOW)
	else:
		if direction == Dir_backward:
			GPIO.output(Motor_B_Pin1, GPIO.LOW)
			GPIO.output(Motor_B_Pin2, GPIO.HIGH)
			pwm_B.start(100)
			pwm_B.ChangeDutyCycle(speed)
		elif direction == Dir_forward:
			GPIO.output(Motor_B_Pin1, GPIO.HIGH)
			GPIO.output(Motor_B_Pin2, GPIO.LOW)
			pwm_B.start(0)
			pwm_B.ChangeDutyCycle(speed)


def motor_right(status, direction, speed):#Motor 1 positive and negative rotation
	if status == 0: # stop
		GPIO.output(Motor_A_Pin1, GPIO.LOW)
		GPIO.output(Motor_A_Pin2, GPIO.LOW)
		GPIO.output(Motor_A_EN, GPIO.LOW)
	else:
		if direction == Dir_forward:#
			GPIO.output(Motor_A_Pin1, GPIO.LOW)
			GPIO.output(Motor_A_Pin2, GPIO.HIGH)
			pwm_A.start(100)
			pwm_A.ChangeDutyCycle(speed)
		elif direction == Dir_backward:
			GPIO.output(Motor_A_Pin1, GPIO.HIGH)
			GPIO.output(Motor_A_Pin2, GPIO.LOW)
			pwm_A.start(0)
			pwm_A.ChangeDutyCycle(speed)



def move(speed, direction, turn, radius):   # 0 < radius <= 1  

	if direction == 'forward':
		if turn == 'left':
			motor_left(1, left_backward, int(speed*radius))
			motor_right(1, right_forward, speed)
		elif turn == 'right':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, int(speed*radius))
		else:
			motor_left(1, left_forward, speed)
			motor_right(1, right_forward, speed)
	elif direction == 'backward':
		if turn == 'right':
			motor_left(1, left_forward, int(speed*radius))
			motor_right(1, right_backward, speed)
		elif turn == 'left':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, int(speed*radius))
		else:
			motor_left(1, left_backward, speed)
			motor_right(1, right_backward, speed)
	elif direction == 'NO':
		if turn == 'right':
			motor_left(1, left_backward, speed)
			motor_right(1, right_forward, speed)
		elif turn == 'left':
			motor_left(1, left_forward, speed)
			motor_right(1, right_backward, speed)
		else:
			motorStop()
	else:
		pass




def destroy():
	motorStop()
	GPIO.cleanup()   


def checkdist():
	GPIO.output(Tr, GPIO.HIGH)
	time.sleep(0.00001)
	GPIO.output(Tr,GPIO.LOW)
	while not GPIO.input(Ec):
		pass

	t1 =  time.time()
	while GPIO.input(Ec):
		pass
	t2 = time.time()
	
	return "{}".format(((t2 -t1)* 320))



def Sonic_net():
	context = zmq.Context()
	socket = context.socket(zmq.REQ)
	port = "5556"

	socket.connect("tcp://localhost:%s" %port)

	while True:
		Sonic_read = checkdist()
		socket.send_string(Sonic_read)
		core_request = socket.recv()
		#print(core_request)
		core_request = core_request.decode('utf-8')
		if core_request != "UltraSonic -r":
			print("Connection failed\n")
			print(core_request)
			return
		time.sleep(0.05)



def convert(motor_command):
    move_command = []
    motor_command = motor_command.decode('utf-8')
	
    values = re.findall('[0-9]+', motor_command)
    words = re.findall(r'\w+', motor_command)
    fvalues = re.findall("[+-]?\d+\.\d+", motor_command)
    #idiocy
    move_command.append(str(values[0]))
    move_command.append(words[1])
    move_command.append(words[2])
    move_command.append(str(fvalues[0]))
    move_command.append(str(fvalues[1]))

    return move_command


def Motor_command():
	port = 5567
	context = zmq.Context()
	socket = context.socket(zmq.REP)
	socket.connect("tcp://localhost:%s" %port)
	
	while True:
		motor_command = convert(socket.recv())
		if len(motor_command) != 5:
			socket.send_string("Format Err")
			break # Signal back to C++ for the critical error
		else:
			print("Running - Move Command ", motor_command)
			move(int(motor_command[0]), motor_command[1], motor_command[2], float(motor_command[3]))
			time.sleep(float(motor_command[4]))

			motorStop()#Hope this would do the trick
			print("\nMove Command Done!\n")
		socket.send_string("GG")

def motorStop():#Motor stops
	GPIO.output(Motor_A_Pin1, GPIO.LOW)
	GPIO.output(Motor_A_Pin2, GPIO.LOW)
	GPIO.output(Motor_B_Pin1, GPIO.LOW)
	GPIO.output(Motor_B_Pin2, GPIO.LOW)
	GPIO.output(Motor_A_EN, GPIO.LOW)
	GPIO.output(Motor_B_EN, GPIO.LOW)



def Head_bop():
	pos_max = 255
	pos_min = 320

	port = 5558
	context = zmq.Context()
	socket = context.socket(zmq.REP)

	socket.connect("tcp://localhost:%s" %port)

	while True:
		Seek_command = socket.recv()
		Seek_command = Seek_command.decode('utf-8')

		if Seek_command == "Observe!":
			for i in range (pos_min,pos_max, -1):
				time.sleep(0.005)
				pwm.set_pwm(11, 0, i)

			time.sleep(1.1)
			#85 NO right 0.400000 0.350000
			move(85, "NO", "right", "0.40")
			time.sleep(0.45)
			motorStop()
			time.sleep(0.4)
			move(85, "NO", "left", "0.40")
			time.sleep(0.90)
			motorStop()
			time.sleep(0.4)
			move(85, "NO", "right", "0.40")
			time.sleep(0.45)
			motorStop()
			
			for i in range (pos_max, pos_min+15):
				pwm.set_pwm(11, 0,i)
				time.sleep(0.005) 

		else:
			socket.send_string("Instruction - Out of Range")
			exit()

		socket.send_string("Seek Done!")
# How about c++ & Python to exchange process IDs so they can communicate then mode is changed ?

		



if __name__ == '__main__':
		
	setup()

	thread_MotorControl = threading.Thread(target = Motor_command)
	thread_UltraSonic = threading.Thread(target = Sonic_net)
	thread_Headbop = threading.Thread(target = Head_bop)

	thread_MotorControl.start()
	thread_UltraSonic.start()
	thread_Headbop.start()

	thread_UltraSonic.join()
	thread_MotorControl.join()
	thread_Headbop.join()
	
	destroy()