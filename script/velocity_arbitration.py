#!/usr/bin/env python3
# license removed for brevity
import copy
# import moveit_commander
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32

class arbitration_utils:
    def __init__(self):
        print("init params")

        self.alpha = 0.8
        self.pub = rospy.Publisher('/alpha', Float32, queue_size=10)
        self.current_vel = TwistStamped()

    def compute_alpha(self,x):

        x_min = 0.05

        if x <0:
            ret = x_min
        # elif 0<x<1:
        #     ret = x_min + 2*x
        elif 0.05<x<1:
            ret = x_min + 2*x
        else:
            ret = 1

        return ret

    def callback(self,data):
        self.current_vel = data
        vl = np.array([self.current_vel.twist.linear.x,self.current_vel.twist.linear.y,self.current_vel.twist.linear.z])
        vnorm = np.linalg.norm(vl)

        self.alpha = self.compute_alpha(vnorm)


    def listener(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("current_vel", TwistStamped, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()




def node():

    rospy.init_node('arbitration_vel_node')

    print("init node")
    au = arbitration_utils()
    current_sub = rospy.Subscriber("/current_velocity", TwistStamped, au.callback)

    rate = rospy.Rate(125)  # 10hz
    while not rospy.is_shutdown():
        au.pub.publish(au.alpha)
        rate.sleep()






if __name__ == '__main__':
    node()
