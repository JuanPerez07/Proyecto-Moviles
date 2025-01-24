"""
Computes all the vision algorithms and publishes info to different topics
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
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Twist

from segmentar import Recon

OBS_TOPIC = '/obstacle'
TARGET_TOPIC = '/target'
IMG_TOPIC = '/camera/rgb/image_raw'
PERSON_TOPIC = '/person_detected'
COMMAND_TOPIC = '/command'
RESOLUTION = (320,210)
#class Tracking(State): #Recognize and sends commands to follow
class Tracking:
    def __init__(self, color, img_topic=IMG_TOPIC, person_topic=PERSON_TOPIC, command_topic=COMMAND_TOPIC):
        self.bridge = cv_bridge.CvBridge()
        # publishers of person detection and command sent to the robot
        self.personPub = rospy.Publisher(person_topic, String, queue_size=5)
        self.commandPub = rospy.Publisher(command_topic, String, queue_size=5)
        # sub to image of the camera from the waffle
        self.imgSub = rospy.Subscriber(img_topic, Image, self.frame_cb)
        # color to track
        self.color = color
        # obstacles checker 
        self.obstaclesPub = rospy.Publisher(OBS_TOPIC, String, queue_size=5)
        # target direction publisher (Float32)
        self.pubTarget = rospy.Publisher(TARGET_TOPIC, Float32, queue_size=8)

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
        # publish to the topic of obstacles to avoid any
        if rec_person.floor_plane_obstacles():
            self.obstaclesPub.publish("true")
        else: 			
            self.obstaclesPub.publish("false")
        # steering direction to folow the target
        direction = 0
        if bbox_info[0][0] != 0 and bbox_info[1][0] != 0: # there is a bounding box
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
            if x == width // 2: # direction must be 0 degrees
                self.pubTarget.publish(direction)			    	
            else:
                if x < width // 2:
                    direction = (x // width) * -90					
                    self.pubTarget.publish(direction)
                else:
                    direction = (x // width) * 90					
                    self.pubTarget.publish(direction)									
        else: # lost the target to follow
            self.pubTarget.publish(direction)
            self.commandPub.publish("STOP")
            self.personPub.publish("not_detected")
        

if __name__ == '__main__':
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
    tracker = Tracking(track_color)

    rospy.spin()
