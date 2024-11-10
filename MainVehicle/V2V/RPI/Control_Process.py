import time
import math
import sysv_ipc
import RPi.GPIO as GPIO
import struct
import os

MAX_SPEED = 100
NORMAL_SPEED = 50

# GPIO setup
ALARM_GPIO = 27         #ALARM   ----> |(BCM) 26  | GPIO 26
MOTOR1_PWM = 13         #L_MOTOR ----> |(BCM) 13  | GPIO 13(PWM1)
MOTOR1_IN1_GPIO = 6     #L_MOTOR ----> |(BCM) 17  | GPIO 17
MOTOR1_IN2_GPIO = 26    #L_MOTOR ----> |(BCM) 22  | GPIO 22
MOTOR2_PWM = 18         #R_MOTOR ----> |(BCM) 18  | GPIO 18(PWM0)
MOTOR2_IN1_GPIO = 22    #R_MOTOR ----> |(BCM) 23  | GPIO 23
MOTOR2_IN2_GPIO = 17    #R_MOTOR ----> |(BCM) 24  | GPIO 24
SERVO_GPIO = 12         #SERVO_M ----> |(BCM) 12  | GPIO 12(PWM0)
FRONT_RIGHT_LED =  24
FRONT_LEFT_LED = 23
BACK_RIGHT_LED = 21
BACK_LEFT_LED = 20

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # Disable GPIO warnings
GPIO.setup(ALARM_GPIO, GPIO.OUT)
GPIO.setup(MOTOR1_IN1_GPIO, GPIO.OUT)
GPIO.setup(MOTOR1_IN2_GPIO, GPIO.OUT)
GPIO.setup(MOTOR2_IN1_GPIO, GPIO.OUT)
GPIO.setup(MOTOR2_IN2_GPIO, GPIO.OUT)
GPIO.setup(SERVO_GPIO, GPIO.OUT)
GPIO.setup(MOTOR2_PWM, GPIO.OUT)
GPIO.setup(MOTOR1_PWM, GPIO.OUT)
GPIO.setup(BACK_LEFT_LED,GPIO.OUT)
GPIO.setup(BACK_RIGHT_LED,GPIO.OUT)
GPIO.setup(FRONT_LEFT_LED,GPIO.OUT)
GPIO.setup(FRONT_RIGHT_LED,GPIO.OUT)

# Initialize PWM for motors
pwm1 = GPIO.PWM(MOTOR1_PWM, 100)  # Motor1 PWM at 100Hz frequency
pwm2 = GPIO.PWM(MOTOR2_PWM, 100)  # Motor2 PWM at 100Hz frequency
pwm1.start(0)
pwm2.start(0)

# Define constants for the servo
MIN_DUTY_CYCLE = 2.5  # Duty cycle for 0 degrees
MAX_DUTY_CYCLE = 12.5  # Duty cycle for 180 degrees
SERVO_ANGLE_RANGE = 180  # Range of the servo in degrees
SERVO_ANGLE_OFFSET = 0  # Offset for the servo angle

# Initialize PWM for servo motor
servo_pwm = GPIO.PWM(SERVO_GPIO, 50)  # 50Hz frequency for servo
servo_pwm.start(0)

# Assuming you have a queue created with sysv_ipc
MODEL_key_receive=12
Motor_key_receive =2
Motor_key_send =3
Servo_key = 4  # Example key

# Variables for encoder
velocity = 0
previous_servo_angle = 0
Alram_Delay = 0

DANGER = 2
FOCUS = 3
NORMAL = 1 

motor_queue_receive = sysv_ipc.MessageQueue(Motor_key_receive, sysv_ipc.IPC_CREAT)
motor_queue_send = sysv_ipc.MessageQueue(Motor_key_send, sysv_ipc.IPC_CREAT)
MODEL_queue_receive = sysv_ipc.MessageQueue(MODEL_key_receive, sysv_ipc.IPC_CREAT)
Servo_queue = sysv_ipc.MessageQueue(Servo_key, sysv_ipc.IPC_CREAT)

def cleanupfirst(Q):

    pwm1.ChangeDutyCycle(0)
    pwm2.ChangeDutyCycle(0)
    GPIO.output(MOTOR2_IN1_GPIO, GPIO.LOW)
    GPIO.output(MOTOR2_IN2_GPIO, GPIO.LOW)
    GPIO.output(MOTOR1_IN2_GPIO, GPIO.LOW)
    GPIO.output(MOTOR1_IN1_GPIO, GPIO.LOW)
    GPIO.output(ALARM_GPIO, GPIO.LOW)
    GPIO.output(BACK_LEFT_LED, GPIO.LOW)
    GPIO.output(BACK_RIGHT_LED, GPIO.LOW)
    GPIO.output(FRONT_LEFT_LED, GPIO.LOW)
    GPIO.output(FRONT_RIGHT_LED, GPIO.LOW)
    # Close and remove the message queue
    Q.remove()
    print('Cleaned up GPIO and removed V2V Queue')

def Led_Lights(which):
    """
    Controls the state of LED lights based on the input parameter `which`.
    - 9 : Turns on the front right LED.
    - 3 : Turns on the back right LED.
    - 7 : Turns on the front left LED.
    - 1 : Turns on the back left LED.
    - 0 : Turns off all LEDs.
    """
    # Turn on the front right LED if `which` is 9
    if which == 9:
        GPIO.output(FRONT_RIGHT_LED, GPIO.HIGH)
    # Turn on the back right LED if `which` is 3
    elif which == 3:
        GPIO.output(BACK_RIGHT_LED, GPIO.HIGH)
    # Turn on the front left LED if `which` is 7
    elif which == 7:
        GPIO.output(FRONT_LEFT_LED, GPIO.HIGH)
    # Turn on the back left LED if `which` is 1
    elif which == 1:
        GPIO.output(BACK_LEFT_LED, GPIO.HIGH)
    # Turn off all LEDs if `which` is 0
    elif which == 0:
        GPIO.output(BACK_LEFT_LED, GPIO.LOW)  # Turn off the back left LED
        GPIO.output(BACK_RIGHT_LED, GPIO.LOW)  # Turn off the back right LED
        GPIO.output(FRONT_LEFT_LED, GPIO.LOW)  # Turn off the front left LED
        GPIO.output(FRONT_RIGHT_LED, GPIO.LOW)  # Turn off the front right LED

def stop_motors():
    pwm1.stop()
    pwm2.stop()

def alarm_control(state):
    GPIO.output(ALARM_GPIO, state)

def motor_forward(speed):
    GPIO.output(MOTOR1_IN1_GPIO, GPIO.HIGH)
    GPIO.output(MOTOR1_IN2_GPIO, GPIO.LOW)
    GPIO.output(MOTOR2_IN1_GPIO, GPIO.HIGH)
    GPIO.output(MOTOR2_IN2_GPIO, GPIO.LOW)
    if speed > 100 :
        speed = 100
    elif speed < 0 :
        speed = 0     
    pwm1.ChangeDutyCycle(int(speed))
    pwm2.ChangeDutyCycle(int(speed))
    if speed <= 5 :
        Led_Lights(0)
        Led_Lights(1)
        Led_Lights(3)
        alarm_control(0)
    else :
        Led_Lights(0)
        Led_Lights(7)
        Led_Lights(9)

def motor_backward(speed):
    GPIO.output(MOTOR1_IN1_GPIO, GPIO.LOW)
    GPIO.output(MOTOR1_IN2_GPIO, GPIO.HIGH)
    GPIO.output(MOTOR2_IN1_GPIO, GPIO.LOW)
    GPIO.output(MOTOR2_IN2_GPIO, GPIO.HIGH)
    print(f'in MOVE : {int(speed)}')
    pwm1.ChangeDutyCycle(int(speed))
    pwm2.ChangeDutyCycle(int(speed))

def V2V_Taking_Actions(V2V_Recommendation,Action_type):
    global velocity
    global Alram_Delay

    # print(f'C -> PY : {V2V_Recommendation}')
    velocity = V2V_Recommendation
    motor_forward(velocity)  
    if (Action_type == DANGER or Action_type == FOCUS) and Alram_Delay < 100:
        # alarm_control(GPIO.HIGH8=979
        Alram_Delay = Alram_Delay + 1
        print("ALARM")

    elif Action_type == NORMAL or Alram_Delay == 100:  
        # alarm_control(GPIO.LOW)
        print("NO ALARM")

def angle_mapping_for_lane(angle, side):
    """
    Calculate the steering angle for a lane keeping system.
    Returns:
    int: The calculated steering angle. Returns -1 if the side parameter is invalid.
    """
    if side == 'l':    # Vehicle is on the right side of the lane
        return angle
    elif side == 'r':  # Vehicle is on the left side of the lane
        return 180 - angle
    else:
        return -1      # Invalid side parameter
    
    # for testing purposes only 
    
    if side == 'l':    # Vehicle is on the right side of the lane
        return 45
    elif side == 'r':  # Vehicle is on the left side of the lane
        return 135
    else:
        return -1      # Invalid side parameter
    
def set_servo_angle(angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    # Convert angle to duty cycle
    duty_cycle = MIN_DUTY_CYCLE + ((angle + SERVO_ANGLE_OFFSET) / SERVO_ANGLE_RANGE) * (MAX_DUTY_CYCLE - MIN_DUTY_CYCLE)
    
    # Set the duty cycle
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)  # Wait for the servo to reach the position
    servo_pwm.ChangeDutyCycle(0)  # Stop sending PWM signal to stop jittering

def process_detected_objects(detected_objects_info):
    parts = detected_objects_info.split('/')

    # Check if parts list has the correct number of elements
    if len(parts) != 4:
        raise ValueError("Invalid format: Expected 4 parts separated by '/'")
    
    num_objects = int(parts[0])
    object_types = parts[1].split(',')
    object_angles_raw = parts[2].split(',')
    object_distances_raw = parts[3].split(',')
    # Filter out empty strings before converting to float
    object_distances = [float(distance) for distance in object_distances_raw if distance]
    object_angles = [float(angle) for angle in object_angles_raw if angle]
    
    # Log parts for debugging
    print(f"num_objects : {num_objects}")
    print(f"object_types : {object_types}")
    # Log filtered angles and distances
    print(f"in range object_angles: {object_angles}")
    print(f"in range object_distances: {object_distances}")
    
    if not (num_objects == len(object_types) == len(object_angles) == len(object_distances)):
        if num_objects != 0:
            print("Mismatch in the number of detected objects and their details")
    
    objects_info = []
    for i in range(num_objects):
        objects_info.append((object_types[i], object_angles[i], object_distances[i]))
    
    return objects_info

def Object_Detection_Recommendations(detected_objects_info):

    global velocity
    objects_info = process_detected_objects(detected_objects_info)

    # Filter objects with angles less than 10 degrees
    relevant_objects = [(obj_type, angle, distance) for obj_type, angle, distance in objects_info if (angle > 60 and angle < 120)]

    # If no objects with angle < 10, return 'idle'
    if not relevant_objects:
        alarm_control(GPIO.LOW)

    # Check distances of relevant objects
    for obj_type, angle, distance in relevant_objects:
        if distance <= 400:
            velocity -= 15
            motor_forward(velocity)
            alarm_control(GPIO.LOW)
        elif 400 < distance <= 800:
            alarm_control(GPIO.HIGH)

def Traffic_Recommendations(traffic_sign_info):
    parts = traffic_sign_info.split('/')
    sign_light = parts[0].split(',')
    traffic_sign=sign_light[0]
    traffic_light=sign_light[1]
    recommendation=parts[1]

    valid_traffic_lights = {'red', 'green', 'yellow'}

    if traffic_light not in valid_traffic_lights:
        print(f"Invalid traffic light: {traffic_light}")
    else :
        print(traffic_light,traffic_sign,recommendation)

    if recommendation == 'STOP':
        stop_motors()

def receive_Model_data(data_queue):
    """
    Expected message format: 
    "Angle between mycar and lane center / Side / Road Anlge/ Road Direction & NO.objects / type1,type2,.../ angle1,angle2,... / distance1,distance2,... & Traffic sign,light / Recommendation"
    Example message:
    "15.0/l&2/car,bike/30.0,45.0/100.0,150.0&STOP,red/stop"
    """
    global velocity
    # Receive the data
    message, mtype = data_queue.receive()
    
    # Decode the message if it's in bytes
    message = message.decode('utf-8')
    # print(message)
    # Split the message into its respective parts
    parts = message.split('&')
    return parts[0],parts[1],parts[2]

def process_lane_info(lane_info):
    try:
        # subparts = lane_info.split('&')
        # Split the input string by '/'
        Car_lane_Angle, side, road_angle, road_direction = lane_info.split('/')
        
        # Convert the angle values from bytes to float
        Car_lane_Angle = float(Car_lane_Angle)
        road_angle = float(road_angle)
        
        # Ensure that the side and road_direction values are valid
        if side not in ('l', 'r'):
            print(f"Invalid side direction: {side}")
        if road_direction not in ('l', 'r'):
            print(f"Invalid road direction: {road_direction}")
        
        return Car_lane_Angle, side, road_angle, road_direction
    except ValueError as e:
        # Handle conversion errors or invalid values
        print(f"Error processing lane info: {e}")
        return None, None, None, None   

def Lane_Detection_Recommendations(ServoData):
    global velocity
    global previous_servo_angle
    print(f'Lane Detection Model : {ServoData}')
    CarLaneAngle,mySide_inlane,RoadAngle,RoadDirection=process_lane_info(ServoData)
    # print(mySide_inlane,CarLaneAngle,RoadAngle,RoadDirection)
    servoAngle=angle_mapping_for_lane(CarLaneAngle,mySide_inlane)
    print(f'ServoAngle : {servoAngle}') 
    if abs(previous_servo_angle - servoAngle)>15:
        print('NEW servo angle')
        set_servo_angle(servoAngle)
    previous_servo_angle=servoAngle
    if (servoAngle >= 115 or servoAngle >= 65) and velocity >= 75 :
        velocity /=2
        motor_forward(velocity) 

def testing_HW(which):
    
    if which == 'm':
        s = int(input('enter speed : '))
        motor_forward(s)
    elif which == 'a':
        s = int(input('enter alarm state : [0,1] '))
        alarm_control(s)
    elif which == 's':
        s = int(input('inter the angle : '))
        set_servo_angle(s)
    elif which == 'l':
        s=int(input('Which led to flash on : \n[ 1 ] is back left\n[ 3 ] is back right\n[ 7 ] is front left\n[ 9 ] is front right\n[ 0 ] all off : '))
        Led_Lights(s)  
    elif which =='b' :
        s=int(input('move backward : '))
        motor_backward(s)

def Receive_Calculation_msg():
    motor_queue_receive = sysv_ipc.MessageQueue(Motor_key_receive, sysv_ipc.IPC_CREAT)
    message, mtype = motor_queue_receive.receive()
    # Interpret the received data as a single uint8_t value
    received_value = struct.unpack('H', message)
    ActualData = received_value[0]
    # print(f'{ActualData},{mtype}')
    return ActualData,mtype

def Start_Code():
    motor_queue_receive = sysv_ipc.MessageQueue(Motor_key_receive, sysv_ipc.IPC_CREAT)
    # Receive the data
    IsStarting = False
    print('python is waition to start')
    while IsStarting != True:
        message, mtype = motor_queue_receive.receive()
        if len(message) != 2:
            print(f"Error: Expected 2 bytes, but received {len(message)} bytes.")
        else :
            received_value = struct.unpack('H', message)
            IsStarting = received_value[0]
    print('RUNNING PY CODE')

def Which_System():
    which = input('Run Code OR Test components? r/t ').lower().strip()[0]
    System =''
    while True:
        if which == 'r':
            System = input("Will you Run V2V process or AI process or both ? -> type V for V2V , A for AI , S for whole System : ").lower().strip()[0] 
            return which,System
        elif which == 't':    
            System = input('Which Actuator you want to test : a for alarm , m for motor , s for servo ,l for leds , b for back : ')
            return which,System
        else :
            which = input('Run Code OR Test components? r/t ').lower().strip()[0]

# Main loop
try:
    cleanupfirst(motor_queue_receive)
    System,Specific = Which_System()
# V2V System
    if Specific == 'v':
        print('V2V System')
#AI Models        
    elif Specific == 'a':
        print('AI System')
#Whole System
    elif Specific ==  's':
        print('To Be Done ... ')

    while True:
        if System =='t':
            testing_HW(Specific)
        elif System =='r':
        # V2V System
            if Specific == 'v':
                GivenData,Action_Type=Receive_Calculation_msg()
                V2V_Taking_Actions(int(GivenData),int(Action_Type))
        #AI Models        
            elif Specific == 'a':
                velocity = NORMAL_SPEED
                motor_forward(velocity)
                Lane,Obj,Traffic=receive_Model_data(MODEL_queue_receive)
                Lane_Detection_Recommendations(Lane)
                Object_Detection_Recommendations(Obj)
                Traffic_Recommendations(Traffic)
        #Whole System
            elif Specific ==  's':
                print('To Be Done ... ')
            
      
except KeyboardInterrupt:
    print('cleanup')
    cleanupfirst(motor_queue_receive)
