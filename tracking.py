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
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

from segmentar import Recon
OBS_TOPIC = '/obstacle'
TARGET_TOPIC = '/target'
RESOLUTION = (320,210)
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
        # obstacles checker 
        self.obstaclesPub = rospy.Publisher(OBS_TOPIC, String, queue_size=5)
        # direction publisher (Int32)
        self.pubTarget = rospy.Publisher(TARGET_TOPIC, Int32, queue_size=8)

    # callback of the frame received
    def frame_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # object Recon to identify person to follow
        rec_person = Recon(self.color)
        # set the given frame
        rec_person.setFrame(frame)
        # resize the current frame
        rec_person.setFrame(rec_person.resize(RESOLUTION))
        # get the bbox info
        bbox_info = rec_person.read_from_wafflecam() # returns the bbox information of the detection -> numpy array [pos, size]
        # matching = rec_person.match_query(bbox_frame) # checks if there is a match with the query
        
        # publish to the topic of obstacles to avoid any
        if rec_person.floor_plane_obstacles():
            self.obstaclesPub.publish("true")
        else: 			
            self.obstaclesPub.publish("false")
        
#       if matching is correct then person is detected
        if bbox_info[0][0] != 0 and bbox_info[1][0] != 0: # person is detected
            self.personPub.publish("detected")
            # size of the bbox
            x0, y0 = bbox_info[0]
            x1, y1 = bbox_info[1]
            # center of the bbox
            x, y = (x0 + x1) / 2, (y0 + y1) / 2
            
            # height and width of the frame
            height, width, _ = rec_person.getFrameShape()
            if x < width // 3: # bbox center on the left side of the frame
                self.commandPub.publish("LEFT")
            elif x > 2*width // 3: # bbox center on the right side of the frame
                self.commandPub.publish("RIGHT")
            else:
                self.commandPub.publish("GO")
            # publish the bbox target direction according to a 180 FOV
            direction = 0
            if x == width // 2:
                self.pubTarget.publish(direction)			    	
            else:
                if x < width // 2:
                    direction = (x // width) * -90					
                    self.pubTarget.publish(direction)
                else:
                    direction = (x // width) * 90					
                    self.pubTarget.publish(direction)									
        else:
            self.commandPub.publish("STOP")
            self.personPub.publish("not_detected")
        

if __name__ == '__main__':
    # topics involved
    speed_topic = "/cmd_vel"
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
        default="yellow",  # default color
        help="Color a rastrear (opciones: 'red', 'yellow', 'green')",
    )
    args = parser.parse_args()

    # read color from command line
    track_color = args.track_color.lower()
    if track_color not in ["red", "yellow", "green"]:
        print("Color not soported. Try 'red' | 'yellow' | 'green'")
    else:
        print("Tracking color: ",  track_color)

    # create object of type Tracking
    tracker = Tracking(track_color, img_topic, person_topic, command_topic)

    rospy.spin()
