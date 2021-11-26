#! /usr/bin/env python
import sys
import rospy
from check2.msg import my
import RPi.GPIO as GPIO
import time


touch1 = 31 # on top
touch2 = 30 # on top
touch3 = 29 # on bottom

def callback(data):
    global now_encoder_data,IR_value
    now_encoder_data = data.encoder
    IR_value         = data.IR_value
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def go_foward(linear,angular):
    speed = my()
    if linear> 20:linear= 20
    if linear<-20:linear=-20
    if angular>  5:angular=  5
    if angular< -5:angular= -5 

    speed.linear = linear
    speed.angular = -1.0*angular
    pub.publish(speed)

def search_ball():
    ''' Spin and record the IR value'''
    global now_encoder_data,IR_value
    encoder_data_ori = now_encoder_data # recode now encoder value
    go_foward(0,5)   # spin
    data_return = dict()
    while (encoder_data_ori + 1920)!= now_encoder_data :
        if (now_encoder_data-encoder_data_ori)%192==0:
            data_return[now_encoder_data-encoder_data_ori]=(IR_value)
    go_foward(0,0)
    return data_return

def decide_anglue(value_matrix):
    ''' Find max IR vlaue index '''
    value=min(value_matrix, key=value_matrix.get)
    return value

def spin_to_desired(value):
    ''' Turn to light ball orientation '''
    encoder_data_ori = now_encoder_data # recode now encoder value
    go_foward(0,5)   # spin
    while value >= now_encoder_data-encoder_data_ori:
        pass
    go_foward(0,0)

def talker():
    global interrupt_en
    if not interrupt_en:
        go_foward(10,0)        # go forward
        if GPIO.input(touch3) == True:
            go_foward(0,0) # stop
    else :
        go_foward(-10,0) # go backwoad
        time.sleep(0.5)

        data_store=search_ball()

        orientation = decide_anglue(data_store)
        spin_to_desired(orientation)
        interrupt_en = 0

def touch_pressed_callback():
    global interrupt_en
    interrupt_en = 1
    
    print("touch pressed!")
    
if __name__ =="__main__":
    global pub
    #GPIO.setmode(GPIO.BOARD)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(touch1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(touch1, GPIO.FALLING,callback=touch_pressed_callback, bouncetime=100)

    GPIO.setup(touch2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(touch2, GPIO.FALLING,callback=touch_pressed_callback, bouncetime=100)

    GPIO.setup(touch3, GPIO.IN)

    rospy.Subscriber("get_sensor", my, callback)
    pub = rospy.Publisher('get_num',my,queue_size=10)
    rospy.init_node("talk",anonymous=True)
    while 1:
        try:
            talker()
        except rospy.ROSInterruprException:
            pass

