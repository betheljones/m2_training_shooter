#!/usr/bin/env python
from adafruit_servokit import ServoKit
import rospy
from std_msgs.msg import Int32

 
def callback_gen(callback_id):
    def cb(msg):
        kit.servo[callback_id].angle = msg.data
        rospy.loginfo(f"setting servo {callback_id} to {msg.data}")
    return cb

if __name__ == "__main__":
    kit = ServoKit(channels=16)
    rospy.init_node("servo_node")
    for i in range(8):
        rospy.Subscriber("servo/"+str(i), Int32, callback_gen(i))
    rospy.spin()
