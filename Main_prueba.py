#!/usr/bin/env python


#from Returning_State import Returning_State
from Initial_State import Waiting_State
from Following_State import FollowingState
from Finding_State import FindingState 
from Delivery_State import DeliveryState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach_ros import SimpleActionState
import tf
import smach_ros
from smach import StateMachine
import rospy
from geometry_msgs.msg import PointStamped
import subprocess
#from find_transform import get_trans,transform_point

#transformation = get_trans()
#home_coord_map = transform_point(transformation,0,0)
subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch", "slam_methods:=gmapping"])


#Return home 
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
#Coordenadas para la vuelta a la base
goal.target_pose.pose.position.x = 0 #home_coord_map.point.x
goal.target_pose.pose.position.y = 0 #home_coord_map.point.y
goal.target_pose.pose.position.z = 0.0
#la orientación en z es un quaternion (x,y,z.w), aquí tomanos todo a 0 menos w=1 -> ángulo 0
goal.target_pose.pose.orientation.w = 1


if __name__ == '__main__':

    rospy.init_node("main")
    home = rospy.wait_for_message('/PRUEBA_HOME', PointStamped)
    goal.target_pose.pose.position.x = home.point.x
    goal.target_pose.pose.position.y = home.point.y
    print("***********************************************************")
    sm = StateMachine(outcomes=['succeeded','aborted','preempted','end'])
    with sm:
        StateMachine.add('InitialState', Waiting_State(),transitions={'start_up':'FollowingState'})
        StateMachine.add('FollowingState', FollowingState(),transitions={'Delivering':'DeliveryState','find_person':'FindingState'})
        StateMachine.add('FindingState', FindingState(),transitions={'Following':'FollowingState'})
        StateMachine.add('DeliveryState', DeliveryState(),transitions={'Following':'FollowingState', 'return home':'ReturningState'})
        StateMachine.add('ReturningState', SimpleActionState('move_base',MoveBaseAction,goal=goal),transitions={'succeeded':'end'})

        #TODO: PREGUNTAR A OTTO Averiguar lo de que esté venga a hacer trayectorias
        #TODO: averiguar cómo no trampear trayectorias
        #TODO: añadir find_transform al launch y ver cómo/cuando lanzarlo
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
