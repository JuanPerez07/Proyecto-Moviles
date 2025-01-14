#INITIAL STATE
from smach import State
from std_msgs.msg import String
import rospy 
from NodoMensajes import commands

USER_TOPIC = "USER_INPUT"
STATE_TOPIC = "ACTUAL_STATE"

class Waiting_State(State):

    def __init__(self):

        State.__init__(self, outcomes=['start_up'])
        self.password = commands['Initial State']
        self.correct = False

    def execute(self, ud):
        #Sub to user inputs
        self.prompt = rospy.Subscriber(USER_TOPIC, String, self.order_callback)
        rate = rospy.Rate(10)

        #Publish of the current state
        self.pub = rospy.Publisher(STATE_TOPIC,String,queue_size=10,latch=True)
        self.pub.publish('Initial State')
        
        #Wait for password
        while not self.correct:
            rate.sleep()
        
        #When password is given:
        if self.correct:
            print("Contraseña correcta. Iniciando sistema.")
            #Clean nodes (unsub and delete publisher)
            self.prompt.unregister()
            self.pub = None
        return "start_up"
    
    # Returns True if the password is correct
    def order_callback(self, msg):
        if(msg.data == self.password):
            print('Contraseña correcta, iniciando sistema')
            self.correct = True
    

'''
import tf2_ros
import tf2_geometry_msgs
import rospy

from geometry_msgs.msg import Point, PointStamped

# Parent and Child of the transformation
PARENT_SYS = 'map'
CHILD_SYS = 'odom'

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# Returns the tf between the given systems
def get_trans():
    try:
        trans = tfBuffer.lookup_transform(PARENT_SYS, CHILD_SYS, rospy.Time(0)) 
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
        rospy.logerr("No se ha podido encontrar la transformación")
        return None

# Returns the tf of a given point
def transform_point(trans, x, y):
    [...]
    punto = Point(x,y,0)
    ps = PointStamped(point=punto)

    #"trans" sería la transformación obtenida anteriormente
    punto_trans = tf2_geometry_msgs.do_transform_point(ps, trans)

    return punto_trans


# Con el mapeado corriendo, ejecutar:
# rosrun rqt_tf_tree rqt_tf_tree
# para ver el árbol de tf y sus jerarquias (parent/child)
'''