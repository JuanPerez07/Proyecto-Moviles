# -*- coding: utf-8 -*-
# from __future__ import print_function
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

# class-State Following -> moves the robot using the commands
class Following(State):
    def __init__(self, scan_topic='/scan' , person_topic='/person_detected', command_topic='/command', speed_topic=SPEED_TOPIC):
        State.__init__(self,outcomes=['find_person', 'stop'])
        # subscribers and publishers
        self.speedPub = rospy.Publisher(speed_topic, Twist, queue_size=5)
        self.personSub = rospy.Subscriber(person_topic, String, self.person_cb )
        self.commandSub = rospy.Subscriber(command_topic, String, self.command_cb )
        self.scanSub = rospy.Subscriber(scan_topic, LaserScan, self.laser_cb)
        
        # attributes
        self.person = False
        self.command = None
        # freeWay linear: usar profundidad camara o dist laser frontal
        #TODO freeWay angular: usar desplazamiento bbox
        self.speed = { 
            "freeWay": {"linear_speed": FREEWAY_LINEAR_SPEED , "angular_speed": FREEWAY_ANGULAR_SPEED },
            "obstacle": {"linear_speed": 0.0, "angular_speed": 0.0}
        }
        self.obstacle = False
        self.frontal_ray = None
        self.safety_threshold = SAFETY_DIST # frontal ray safety distance
        # gains to regulate the linear speed and angular speed while following in a freeWay
        self.kp = {"linear": - math.log(SAFETY_DIST) * GAMMA * (FREEWAY_LINEAR_SPEED/10.0) , "angular": 0.9} # speed gains for freeway
        self.freq = FREQ
        self.last_twist = None # last Twist sent to the tb2
    
    def laser_cb(self, msg):
        index = len(msg.ranges) // 2 # index of the frontal ray
        self.frontal_ray = msg.ranges[index] # distance frontal ray
        # linear speed when there is no obstacle based on frontal ray
        #self.speed['freeWay']['linear_speed'] = self.frontal_ray * self.kp['linear']
        """
        # cuales son los rayos de laser que nos interesan?
        pos_izq = int((ANG_IZQ-msg.angle_min)/msg.angle_increment)
        pos_der = int((ANG_DER-msg.angle_min)/msg.angle_increment) 

        # calcular la velocidad angular en z y lineal en x adecuadas en funcion de las distancias
        d_left, d_right = msg.ranges[pos_izq], msg.ranges[pos_der]

        # velocidad angular proporcional a diferencia de distancias entre rayos
        angular_speed = abs(d_left - d_right)
        angular_speed_dir = 1 # direccion de giro antihoraria

        # direccion del giro
        if d_right > d_left and angular_speed > self.safety_threshold:
            angular_speed_dir = -1 # obstaculo proximo a la izq.-> giro sentido horario

        # velocidad lineal proporcional a la media de distancias
        linear_speed = (d_right + d_left)/2
        """
        # centinela para detectar obstaculo
        obs = False
        """
        # si la velocidad lineal es menor que un umbral=1 -> incrementar velocidad angular
        if linear_speed < self.safety_threshold:
            angular_speed =  angular_speed + 2
            linear_speed = 0
            obs = True
        # asignar velocidades lineares y angulares para evitar obstaculos
        self.speed['obstacle']['linear_speed'] = linear_speed
        self.speed['obstacle']['angular_speed'] = angular_speed
        """
        if self.frontal_ray < self.safety_threshold:
            obs = True
        self.obstacle = obs 

    def person_cb(self, msg):
        self.person = msg.data == "detected"
        """
        if msg.data == "detected":
            self.person = True
        else: 
            self.person = False
        """
    def command_cb(self, msg):
        self.command = msg.data

    def execute(self, userdata):
        rate = rospy.Rate(self.freq)
        cmd = Twist()
        while(self.person):
            if not self.obstacle: # si no hay obstaculo seguir a la persona
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
                """
                else:
                    linear_speed = 0.0
                    angular_speed = 0.0
                    print("Wronggo Command!")
                """
            else: # there is obs so we stop
                """
                cmd.linear.x = self.speed['obstacle']['linear_speed']
                cmd.angular.z = self.speed['obstacle']['angular_speed'] """
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            # regulate the speed to avoid bumps due to lost of the target
            if last_twist is None:
                last_twist = cmd
            else: # we already have a previous speed so we regulate it
                if last_twist.linear.x < cmd.linear.x and cmd.linear.x < FREEWAY_LINEAR_SPEED: # increment speed
                    cmd.linear.x = last_twist.linear.x + self.kp['linear']
                if last_twist.linear.x > cmd.linear.x and cmd.linear.x > 0: # decrease
                    cmd.linear.x = last_twist.linear.x - self.kp['linear']
            # publish the speeds
            self.speedPub.publish(cmd)
            rate.sleep()
            """
            # if the frontal ray data is below threshold -> end simulation
            too_close = self.frontal_ray < self.safety_threshold
            if too_close:
                rospy.loginfo("Safety threshold triggered!")
                return 'stop'
            """
        last_twist = None # reset for the next cycle
        # we change State if there is no person
        return 'find_person'
        
# class-State Finding -> makes the robot rotate around itself until he detects the person to then follow
class Finding(State):
    def __init__(self, person_topic='/person_detected', speed_topic=SPEED_TOPIC):
        State.__init__(self, outcomes=['follow'])
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
        
        return 'follow'


"""
MAIN para probar a ejecutar fsm de 2 estados: buscar persona y seguir persona
"""
if __name__ == '__main__':
    # topics involved
    scan_topic = '/scan'
    img_topic='/camera/rgb/image_raw'
    person_topic='/person_detected'
    command_topic='/command'
    # init rospy node
    rospy.init_node("listener")
    sm = StateMachine(outcomes=['end'])
    with sm:
        StateMachine.add('Finding', Finding(), transitions={'follow':'Following'})
        StateMachine.add('Following', Following(), transitions={'find_person':'Finding', 'stop': 'end'})
    
    while not rospy.is_shutdown():
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
        sm.execute()
        rospy.spin()

    '''Speedpub = rospy.Publisher(speed_topic, Twist, queue_size=5)
    tw = Twist()
    tw.linear.x = 0
    tw.angular.z = 0
    Speedpub.publish(tw)'''
