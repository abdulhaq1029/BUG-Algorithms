
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
import math
import time


class dog:
    def __init__(self):
        rospy.init_node('hello')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.stop()
        
    def stop(self):
            rospy.loginfo('stopping ')
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            rate=rospy.Rate(10)
            start = time.time()
            current= time.time()
            while not rospy.is_shutdown() and (current - start)< 10.000000000000000:
                # current = (time.time())
                # print((current - start))
                self.cmd_vel_pub.publish(twist_msg)
                rate.sleep()    


if __name__ =="__main__":
    let = dog()
    