import rospy
import smach_ros
import math
from smach import State, StateMachine
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

ANG_IZQ = 30*math.pi/180.0
ANG_DER = -ANG_IZQ

FREEWAY_LINEAR_SPEED = 0.2
FREEWAY_ANGULAR_SPEED = 0.5
FINDING_ANGULAR_SPEED = 0.25
SAFETY_DIST = 0.75 # secure distance from frontal obstacles
GAMMA = 5 # constant to regulate speed increase/ decrease over time

SPEED_TOPIC = "/mobile_base/commands/velocity"
FREQ = 10 # Hz
        
# class-State Finding -> makes the robot rotate around itself until he detects the person to then follow
class FindingState(State):
    def __init__(self, person_topic='/person_detected', speed_topic=SPEED_TOPIC):
        State.__init__(self, outcomes=['Following'])
        self.found = False
        self.theta = FINDING_ANGULAR_SPEED # angular speed
        self.speedPub = rospy.Publisher(speed_topic, Twist, queue_size=5)
        self.personSub = rospy.Subscriber(person_topic, String, self.person_cb )
        self.freq = FREQ
       
    def person_cb(self, msg):
        self.found = msg.data == "detected"
    
    def execute(self, userdata):
        tw = Twist()
        rate = rospy.Rate(self.freq)
        while(not self.found):
            tw.angular.z = self.theta
            tw.linear.x = 0
            # publish the Twist
            self.speedPub.publish(tw)
            rate.sleep()
        
        return 'Following'
