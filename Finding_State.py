import rospy
import smach_ros
import math
from smach import State, StateMachine
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
from vfh_plus import *

FREEWAY_LINEAR_SPEED = 0.2
FREEWAY_ANGULAR_SPEED = 0.5
FINDING_ANGULAR_SPEED = 0.25
SAFETY_DIST = 0.75 # secure distance from frontal obstacles
GAMMA = 5 # constant to regulate speed increase/ decrease over time
# topics to sub from 
SCAN_TOPIC = '/scan'
SPEED_TOPIC = '/mobile_base/commands/velocity'
TARGET_TOPIC = '/target'
# frequency of reading
FREQ = 10 # Hz
# class-State Finding -> makes the robot rotate around itself until he detects the person to then follow
class FindingState(State):
    def __init__(self, person_topic='/person_detected', speed_topic=SPEED_TOPIC):
        State.__init__(self, outcomes=['Following'])
        self.found = False
        self.theta = 0.0 # angular speed
        self.speedPub = rospy.Publisher(speed_topic, Twist, queue_size=5)
        # subscribers
        self.personSub = rospy.Subscriber(person_topic, String, self.person_cb)
        self.scanSub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.laser_tb2_cb) 
        self.targetSub = rospy.Subscriber(TARGET_TOPIC, Int32, self.target_cb)
        # frecuency of posting
        self.freq = FREQ
       
    def person_cb(self, msg):
        self.found = msg.data == "detected"

    # target direction callback
    def target_cb(self, msg):
        self.target_direction = msg.data
    
    def laser_tb2_cb(self, msg):
    # store only the frontal 180 degrees beam range from right side of the robot to left side 
        reads = []
        n = len(msg.ranges)
        for i in range(90, n - 90):
            reads.append(msg.ranges[i])        			
        # convert to numpy array
        self.laser_readings = np.array(reads)
        self.angle_inc = np.degrees(msg.angle_increment)
        self.max_angle = np.degrees((msg.angle_max * 3) // 2) # half as we use half of the laser beams
            
    def execute(self, userdata):
        #global speed_directions
        rate = rospy.Rate(FREQ)
        tw = Twist()
        last_dir = self.target_direction # suppose it was zero degrees        
        while(not self.found):
	    # compute the initial histogram and plot it
            if self.laser_readings is not None:
                vfh = VFHPlus(self.laser_readings, self.angle_inc, self.max_angle, last_dir)
                vfh.compute_direction()
                vfh.plot_histogram(vfh.hist)
                direct = vfh.getDirection()
                # direction chosen by VFH
                if direct is not None: 
                    last_dir = direct
                    print("Selected direction in degrees: ", np.degrees(direct))
                    """
                    if direct == 0 and speed_directions is not None and len(speed_directions) > 0:
                        omega = speed_directions.pop(0)  						
                        self.theta = - omega # angular speed chosen
                    else:
                    """
                        self.theta = - direct * 0.5 * FREQ
                # no direction chosen so we move to the target
                else:
                    self.theta = - self.target_direction * FREQ * 0.2   					
                
            tw.angular.z = self.theta
            tw.linear.x = 0
            # publish the Twist
            self.speedPub.publish(tw)
            rate.sleep()
        
        # if found we stop the robot 
        tw.angular.z = 0
        tw.linear.x = 0       
        self.speedPub.publish(tw)
        
        return 'Following'
