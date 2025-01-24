#FOLLOWING STATE
#Library imports
import rospy
from std_msgs.msg import String
import time
import subprocess
from Nodo_Interfaz import commands
import smach_ros
import math
from smach import State, StateMachine
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32
from vfh_plus import *
# Movement parameters
FREEWAY_LINEAR_SPEED = 0.2
FREEWAY_ANGULAR_SPEED = 0.5
FINDING_ANGULAR_SPEED = 0.25
SAFETY_DIST = 0.75 # secure distance from frontal obstacles
GAMMA = 5 # constant to regulate speed increase/ decrease over time
REDUCTION_ANG_SPEED= 0.2
# Communication topics
SCAN_TOPIC = '/scan'
COMMAND_TOPIC = '/command'
PERSON_TOPIC = '/person_detected'
SPEED_TOPIC = "/mobile_base/commands/velocity"
OBS_TOPIC='/obstacle'
TARGET_TOPIC='/target'
FREQ = 10 # Hz
# Topics to communicate with the user
TOPIC_NAME = "USER_INPUT"
STATE_TOPIC = "ACTUAL_STATE"

class FollowingState(State):
    def __init__(self):
        State.__init__(self, outcomes=['Delivering','find_person'])

        # subscribers and publishers
        self.speedPub = rospy.Publisher(SPEED_TOPIC, Twist, queue_size=5)
        self.personSub = rospy.Subscriber(PERSON_TOPIC, String, self.person_cb )
        self.commandSub = rospy.Subscriber(COMMAND_TOPIC, String, self.command_cb )
        self.scanSub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.laser_cb)
        self.obsSub = rospy.Subscriber(OBS_TOPIC, String, self.obs_cb)  
        self.targetSub = rospy.Subscriber(TARGET_TOPIC, Float32, self.target_cb)
        self.lastTwistSub = rospy.Subscriber('/cmd_vel', Twist, self.last_velocity_cb)
        # attributes
        self.person = False
        self.command = None
        self.target_direction = 0
        # freeWay linear: usar profundidad camara o dist laser frontal
        #TODO freeWay angular: usar desplazamiento bbox
        self.speed = { 
            "freeWay": {"linear_speed": FREEWAY_LINEAR_SPEED , "angular_speed": FREEWAY_ANGULAR_SPEED },
            "obstacle": {"linear_speed": 0.0, "angular_speed": 0.0}
        }
        self.obstacle = None
        self.safety_threshold = SAFETY_DIST # frontal ray safety distance
        # gains to regulate the linear speed and angular speed while following in a freeWay
        self.kp = {"linear": - math.log(SAFETY_DIST) * GAMMA * (FREEWAY_LINEAR_SPEED/10.0) , "angular": 0.9} # speed gains for freeway
        self.freq = FREQ
        self.last_twist = None # last Twist sent to the tb2
        # laser scan variables
        self.max_angle = None # max angle fo the laser beam
        self.angle_inc = None # angle increment of the laser
        self.laser_readings = None # array of distances read by the scan
        # close other plot 
        plt.close('all')        
        self.correct = False
        self.comandoUser = commands['Following State']

    def last_velocity_cb(self, msg):
        self.last_twist = msg    
    # store only the frontal 180 degrees beam range from right side of the robot to left side 
    def laser_cb(self, msg):
        reads = []
        n = len(msg.ranges)
        for i in range(90, n - 90):
            reads.append(msg.ranges[i])        			
        # convert to numpy array
        self.laser_readings = np.array(reads)
        self.angle_inc = np.degrees(msg.angle_increment)
        self.max_angle = np.degrees((msg.angle_max * 3) // 2) # half as we use half of the laser beams
    # obstacle topic callback
    def obs_cb(self, msg):
        if msg.data == "true":
            self.obstacle = True
        else:
            self.obstacle = False
    # target direction callback
    def target_cb(self, msg):
        self.target_direction = msg.data
    # person detected callback  
    def person_cb(self, msg):
        self.person = msg.data == "detected"
    # user command callback
    def command_cb(self, msg):
        self.command = msg.data
    def order_callback(self,msg):
        if(msg.data == self.comandoUser):
            self.correct = True

    def execute(self,userdata):
        #Launch user prompt subscriptor % state publisher
        self.prompt = rospy.Subscriber(TOPIC_NAME, String, self.order_callback)
        rate = rospy.Rate(10)
        self.pub = rospy.Publisher(STATE_TOPIC,String,queue_size=10,latch=True)
        self.pub.publish("Following State")

        rate = rospy.Rate(self.freq)
        cmd = Twist()
        
        print('Introduzca el comando <<Entrega>> para cambiar de modo')
        self.correct = False
        linear = 0
        #Wait for correct password while following the person
        while not self.correct and self.person:
            # Check for person
            if not self.obstacle: # No obstacle detected
                if self.command == "GO":
                    cmd.linear.x = self.speed['freeWay']['linear_speed']
                    cmd.angular.z = 0.0
                if self.command == "LEFT":
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.speed['freeWay']['angular_speed']
                if self.command == "RIGHT":
                    cmd.linear.x = 0.0
                    cmd.angular.z = - self.speed['freeWay']['angular_speed']
                if self.command == "STOP":
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    
                if self.last_twist is not None: # we already have a previous speed so we regulate it
                    if self.last_twist.linear.x < linear and linear < FREEWAY_LINEAR_SPEED: # increment speed
                        cmd.linear.x = self.last_twist.linear.x + self.kp['linear']
                    if self.last_twist.linear.x > linear and linear > 0: # decrease
                        cmd.linear.x = self.last_twist.linear.x - self.kp['linear']
                else:
                    cmd.linear.x = self.speed['freeWay']['linear_speed']
    
            else: # there is obs so we stop
                omega = 0 # angular velocity
                target = self.target_direction # original target
                last_dir = 0
                #speed_directions = [] # list of inverse speeds taken to regain the original target       
                while(self.obstacle):
                    print("Performing VFH_PLUS")
                    # compute the initial histogram and plot it
                    if self.laser_readings is not None:                       
                        print("Target (degrees) : ", self.target_direction)
                        vfh = VFHPlus(self.laser_readings, self.angle_inc, self.max_angle, last_dir)
                        vfh.compute_direction(self.target_direction)
                        vfh.plot_histogram(vfh.hist)
                        direct = vfh.getDirection() 
                        if direct is not None:
                            last_dir = direct # renew the last direction taken
                            target = - direct # new target as we moved away from it
                            print("Selected direction in degrees: ", np.degrees(direct))
                            omega = - direct * FREQ * REDUCTION_ANG_SPEED # angular speed chosen
                            #speed_directions.append(-omega) # append the opposite direction speed
                        if omega < 0.00001:
                            omega = 0  
                    # assign the speeds        
                    cmd.angular.z = omega
                    
                    if self.last_twist is not None: # we already have a previous speed so we regulate it
                        if self.last_twist.linear.x < self.speed['freeWay']['linear_speed']: # increment speed
                            cmd.linear.x = self.last_twist.linear.x + self.kp['linear']
                        if self.last_twist.linear.x > self.speed['freeWay']['linear_speed']: # decrease
                            cmd.linear.x = self.last_twist.linear.x - self.kp['linear']
                    else:
                        cmd.linear.x = self.speed['freeWay']['linear_speed']
                    # publish speeds
                    self.speedPub.publish(cmd)
            
            rate.sleep()
            self.last_twist = None # reset for the next cycle
        # we change State if there is no person
        
        #Password given, changing to Delivery State
        if self.correct:
            print("Comando recibido. Descansando.")
            #Listener desubscription
            self.prompt.unregister() 
            #Publisher to None
            self.pub = None
            return "Delivering"
        else:
            # Person lost, changing to Finding State
            print('Voy a buscar')
            return 'find_person'


    

   
