#! /usr/bin/env python
import rospy
from check2.msg import my


def talker():
    speed = my()
    a = float(input("enter linear speed "))
    b = float(input("enter angular speed "))

    if a> 20:a= 20
    if a<-20:a=-20
    if b>  5:b=  5
    if b< -5:b= -5 

    speed.linear = a
    speed.angular = -1.0*b
    pub.publish(speed)

    # print("linear: {}, angular: {}".format(a,b))
    print("send\n")
    
if __name__ =="__main__":
    global pub
    
    pub = rospy.Publisher('get_num',my,queue_size=10)
    rospy.init_node("talk",anonymous=True)
    while 1:
        try:
            talker()
        except rospy.ROSInterruprException:
            pass

