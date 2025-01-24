#Funcion de movimiento
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Use for going home: send_goal(0, 0, 1)
def send_goal(x, y, yaw):
    # Initialize the ROS node
    rospy.init_node('send_goal', anonymous=True)
    
    # Create an action client for the move_base server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # Wait for the action server to be ready
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")
    
    # Create a goal to send to move_base
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Reference frame of the map
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Set goal position
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    # Set goal orientation (in quaternion)
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(0, 0, yaw)  # Roll, pitch, yaw
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]
    
    # Send the goal
    rospy.loginfo(f"Sending goal: x={x}, y={y}, yaw={yaw}")
    client.send_goal(goal)
    
    # Wait for the result
    client.wait_for_result()
    result = client.get_state()
    
    if result == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
        return True
    else:
        rospy.logwarn(f"Goal failed with state: {result}")
        return False