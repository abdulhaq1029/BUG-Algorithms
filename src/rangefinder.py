import rospy
from sensor_msgs.msg import LaserScan
import time
class BugZero:
    def __init__(self):
        rospy.init_node('obstacle_detection')
        self.regions_ = {
            'right': float('inf'),
            'fright': float('inf'),
            'front': float('inf'),
            'fleft': float('inf'),
            'left': float('inf'),
        }
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        
        while True:
            self.check_for_obstacle_in_front()
            time.sleep(1)

    def clbk_laser(self, msg):
        # Assuming msg.ranges contains the laser scan data
        # Update the 'front' region with the minimum distance to an obstacle in front
        self.regions_['front'] = min(msg.ranges[288:431], default=float('inf'))

    def check_for_obstacle_in_front(self):
        # Check if there is an obstacle within 0 - 1 meters in front
        if self.regions_['front'] <= 1:
            rospy.loginfo("True")  # Log "True" if there is an obstacle in front
        else:
            rospy.loginfo("False")  # Log "False" otherwise

if __name__ == "__main__":
    bug_zero = BugZero()
    rospy.spin()
