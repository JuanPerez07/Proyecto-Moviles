#INITIAL STATE
#Library imports
from smach import State
from std_msgs.msg import String
import rospy 
from Nodo_Interfaz import commands
#Communication topics
USER_TOPIC = "USER_INPUT"
STATE_TOPIC = "ACTUAL_STATE"

class Waiting_State(State):

    def __init__(self):
        State.__init__(self, outcomes=['start_up'])
        self.password = commands['Initial State']
        self.correct = False

    def execute(self, ud):
        #Subscribe to user inputs
        self.prompt = rospy.Subscriber(USER_TOPIC, String, self.order_callback)
        rate = rospy.Rate(10)

        #Publish the current state
        self.pub = rospy.Publisher(STATE_TOPIC,String,queue_size=10,latch=True)
        self.pub.publish('Initial State')
        
        #Wait for password
        while not self.correct:
            rate.sleep()
        
        #When correct password is given:
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