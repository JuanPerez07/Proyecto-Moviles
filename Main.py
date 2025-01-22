#!/usr/bin/env python
#Library & States imports
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

#Use this line in simulation
#subprocess.Popen(["roslaunch", "turtlebot3_slam", "turtlebot3_slam.launch", "slam_methods:=gmapping"])

#Use this line with the real robot
subprocess.Popen(["rosrun", "rviz", "rviz"])

#Return home 
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = 'map'
#Initialize home coordinates
goal.target_pose.pose.position.x = 0 #home_coord_map.point.x
goal.target_pose.pose.position.y = 0 #home_coord_map.point.y
goal.target_pose.pose.position.z = 0.0
goal.target_pose.pose.orientation.w = 1


if __name__ == '__main__':

    #Main node
    rospy.init_node("main")

    #Wait until the new coordinates are published
    home = rospy.wait_for_message('/PRUEBA_HOME', PointStamped)
    goal.target_pose.pose.position.x = home.point.x
    goal.target_pose.pose.position.y = home.point.y
    sm = StateMachine(outcomes=['succeeded','aborted','preempted','end'])

    #State machine
    with sm:
        StateMachine.add('InitialState', Waiting_State(),transitions={'start_up':'FollowingState'})
        StateMachine.add('FollowingState', FollowingState(),transitions={'Delivering':'DeliveryState','find_person':'FindingState'})
        StateMachine.add('FindingState', FindingState(),transitions={'Following':'FollowingState'})
        StateMachine.add('DeliveryState', DeliveryState(),transitions={'Following':'FollowingState', 'return home':'ReturningState'})
        StateMachine.add('ReturningState', SimpleActionState('move_base',MoveBaseAction,goal=goal),transitions={'succeeded':'end'})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
