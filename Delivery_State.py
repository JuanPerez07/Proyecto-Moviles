#DELIVERY STATE 
from smach import State
import rospy
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import time
import subprocess
import os
import yaml 
from NodoMensajes import commands



USER_TOPIC = "USER_INPUT"
STATE_TOPIC = "ACTUAL_STATE"

SUBFOLDER = "Maps"

class DeliveryState(State):
    def __init__(self):
        State.__init__(self, outcomes=['Following','return home'])
        self.continuar = commands["Delivery State"][0] 
        self.volver = commands["Delivery State"][1]
        self.Flag = False
        self.Flag_Continuar = False
        self.Flag_Volver = False
        self.prompt =''

    def execute(self, ud):
        self.Flag = False
        # Launch subscriptor
        self.prompt = rospy.Subscriber(USER_TOPIC, String, self.order_callback)
        rate = rospy.Rate(10)
        # Launch publisher 
        self.pub = rospy.Publisher(STATE_TOPIC,String,queue_size=10,latch=True)
        self.pub.publish("Delivery State")

        # Save the current map
        self.save_map()
        #print('Introduzca el comando <<Continuar>> para volver al modo de seguimiento \n')
        #print('Introduzca el comando <<Volver>> para volver a la base')

        
        while not self.Flag:
            #print('Repartidor en modo espera...\n')
            rate.sleep()
        
        if self.Flag_Continuar:
            self.pub.publish("Following State")
            return 'Following'
        elif self.Flag_Volver:
            # STOP SLAM 
            self.stop_SLAM()
            #COMENTAR PARA EL ROBOT REAL
            self.prepare_navigation()
            return 'return home'

    # Callback from Subscriber
    def order_callback(self,msg):
        # Authorize transition back to Following state
        if(msg.data == self.continuar):
            self.Flag_Continuar= True
            self.Flag_Volver = False
            self.Flag = True

        # Authorize transition to Return home state
        if(msg.data == self.volver):
            self.Flag_Volver = True
            self.Flag_Continuar = False
            self.Flag = True

    # Reformat the .yaml file
    def format_yaml(self, map_file):
        # Load .yaml file
        try:
            with open(map_file, 'r') as file:
                map_data = yaml.safe_load(file)

            # Modify image header
            map_data['image'] = './mapa.pgm'

            # Dump new header into file
            with open(map_file, 'w') as file:
                yaml.safe_dump(map_data, file, default_flow_style=False)
            
        except FileNotFoundError:
            print(f"Error: The file does not exist.")
        except yaml.YAMLError as e:
            print(f"Error processing YAML file: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")


    # Save the current map
    def save_map(self):
        try:
            rospy.loginfo("Guardando mapa...")
            #  Check directory
            if not os.path.exists(SUBFOLDER):
                os.makedirs(SUBFOLDER)
            
            map_path = os.path.join(SUBFOLDER, "mapa")
            # Map saving command
            subprocess.call(["rosrun", "map_server", "map_saver", "-f",map_path])

            # Reformat the .yaml file
            self.format_yaml('./Maps/mapa.yaml')
            rospy.loginfo("Mapa actualizado exitosamente.")

        except Exception as e:
            rospy.logerr(f"Error en la actualización del mapa: {e}")
      
            
    # Stop SLAM and RVIZ
    def stop_SLAM(self):
        try:
            rospy.loginfo("Deteniendo mapeado...")
            # Command to kill RVIZ
            # Command to stop Rviz, at the moment is optional
            subprocess.call(["killall","-9","rviz"])
            rospy.loginfo("Mapa actualizado exitosamente.")
        except Exception as e:
            rospy.logerr(f"Error en la detención del mapeado: {e}") 


    # Open map in navigation mode
    def prepare_navigation(self):
        try:
            rospy.loginfo("Lanzando mapa de navegacion...")

            #FILE PATH: home/nicolas/ProyectoMoviles_ws/src/navigation_stage/src/Maps/map.yaml
            # Command to open the map
            subprocess.Popen(["roslaunch", "turtlebot3_navigation", "turtlebot3_navigation.launch","map_file:=/home/nicolas/ProyectoMoviles_ws/src/navigation_stage/src/Maps/mapa.yaml"])
            time.sleep(3)
            print('****************************************************************************')
            rospy.loginfo("Mapa actualizado exitosamente.")
        except Exception as e:
            rospy.logerr(f"Error en la detención del mapeado: {e}") 
