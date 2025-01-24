#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry

# Parent and Child of the transformation
PARENT_SYS = 'map'
CHILD_SYS = 'odom'

rospy.init_node('tf_listener_node')
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rospy.sleep(1.0)

# Returns the tf between the given systems
def get_trans():
    try:
        trans = tfBuffer.lookup_transform(PARENT_SYS, CHILD_SYS, rospy.Time(0)) 
        print("success!")
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
        rospy.logerr("No se ha podido encontrar la transformación")
        return None

# Returns the tf of a given point
def transform_point(trans, x, y):
    # Maybe necessary to format the odom msg
    punto = Point(x,y,0)
    ps = PointStamped(point=punto)
    punto_trans = tf2_geometry_msgs.do_transform_point(ps, trans)

    return punto_trans  




#Get initial (home) position from odom and transform
home = rospy.wait_for_message('/odom', Odometry)
home_x = home.pose.pose.position.x
home_y = home.pose.pose.position.y

trans = get_trans()

map_home = transform_point(trans,home_x,home_y)
publicar_home = rospy.Publisher('/PRUEBA_HOME',PointStamped,queue_size=10,latch=True)
publicar_home.publish(map_home)
rospy.sleep(1)
print("publicado")