#!/usr/bin/env python

import rospy
from std_msgs.msg import String

USER_TOPIC = "USER_INPUT"
NODE_NAME = "USER_NODE"
EXIT_MSG = 'salir'
STATE_TOPIC = "ACTUAL_STATE"
CHECK_STATE_MESSAGE = 'state'
ask_input = False
State =""
commands = {'Initial State':'Password','Following State':'Delivery','Delivery State':['Continue','Return'],'Return home':'Return'}


def main():
    global ask_input
    global State
    # Initialize the ROS node
    rospy.init_node(NODE_NAME, anonymous=True)
    
    # Define the publisher
    pub = rospy.Publisher(USER_TOPIC, String, queue_size=10)
    
    # Set the loop rate (10 Hz)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Robot operativo. Esperando órdenes.")
    rospy.Subscriber(STATE_TOPIC,String,print_callback)

    while not rospy.is_shutdown():
        # Get user input
        if ask_input:
            print('ESTADO ACTUAL -> '+ State)       
            user_input = input("Introduzca orden ('salir' para terminar):\n >>")
            # Get machine state

            if user_input.lower() == EXIT_MSG:
                rospy.loginfo("Saliendo...")
                break
            if user_input.lower() == CHECK_STATE_MESSAGE:
                print('Actual State -> ' + State)
            
            # Publish the input as a message
            pub.publish(user_input)
            rospy.loginfo(f"Publicando: <<{user_input}>>")
            #Check if user_input is a registered command
            for key,value in commands.items():
                if key == 'Delivery State' and (user_input == value[0] or user_input == value[1]):
                    ask_input = False
                elif key == State and value == user_input:
                    ask_input = False
        # Sleep to maintain loop rate
        rate.sleep()

def print_callback(msg):
    global ask_input
    global State
    #print('ESTADO ACTUAL -> '+ msg.data)
    ask_input = True
    State = msg.data

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
