"""
Simular tracking segun segmentacion por color en turtlebot waffle
"""
# -*- coding: utf-8 -*-
# from __future__ import print_function
import rospy
import cv2 as cv
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt
import argparse
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from segmentar import Recon

#ANG_IZQ = 30*math.pi/180.0
#ANG_DER = -ANG_IZQ
# color of the floor the robot will walk trough
FLOOR_COLOR = "?"

#class Tracking(State): #Recognize and sends commands to follow
class Tracking:
    def __init__(self, color, img_topic='/camera/rgb/image_raw', person_topic='/person_detected', command_topic='/command'):
        self.bridge = cv_bridge.CvBridge()
        # publishers of person detection and command send to the robot
        self.personPub = rospy.Publisher(person_topic, String, queue_size=5)
        self.commandPub = rospy.Publisher(command_topic, String, queue_size=5)
        # sub to image of the camera from the waffle
        self.imgSub = rospy.Subscriber(img_topic, Image, self.frame_cb)
        # color to track
        self.color = color
        # floor color
        self.floor_color = FLOOR_COLOR

    # callback of the frame received
    def frame_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # object Recon to identify person to follow
        rec_person = Recon(self.color)
        rec_person.setFrame(frame)
        bbox_info = rec_person.read_from_wafflecam() # returns the bbox information of the detection -> numpy array [pos, size]
        # matching = rec_person.match_query(bbox_frame) # checks if there is a match with the query
        #print("bbox: ", bbox_info)
#       if matching is correct then person is detected
        a, b = bbox_info[0]
        if a != 0 and b != 0: # person is detected
            self.personPub.publish("detected")
            # size of the bbox
            x1, y1 = bbox_info[0]
            x2, y2 = bbox_info[1]
            # center of the bbox
            x, y = (x2+x1)/2 , (y2+y1)/2

            # height and width of the frame
            height, width, _ = frame.shape
            #print(frame.shape)
            if x < width // 3: # bbox center on the left side of the frame
                self.commandPub.publish("LEFT")
            if x > 2*width // 3: # bbox center on the right side of the frame
                self.commandPub.publish("RIGHT")
            if x >= width // 3 and x < 2*width //3 :
                self.commandPub.publish("GO")
        else:
            self.commandPub.publish("STOP")
            self.personPub.publish("not_detected")


if __name__ == '__main__':
    # topics involved
    img_topic='/camera/rgb/image_raw'
    person_topic='/person_detected'
    command_topic='/command'
    # start ros node
    rospy.init_node("tracker")
    
    # Configurar el analizador de argumentos
    parser = argparse.ArgumentParser(description="Seguimiento de color con la clase ReconPerson")
    parser.add_argument(
        "--track-color",
        type=str,
        default="yellow",  # Color predeterminado
        help="Color a rastrear (opciones: 'red', 'yellow', 'green')",
    )
    args = parser.parse_args()

    # Leer el color desde los argumentos de línea de comandos
    track_color = args.track_color.lower()
    if track_color not in ["red", "yellow"]:
        print("Color no soportado. Usa 'red' | 'yellow' | 'green'")
    else:
        print(f"Realizando seguimiento al color: {track_color}")

    # create object of type Tracking
    tracker = Tracking(track_color ,img_topic, person_topic, command_topic)
    
    rospy.spin()
