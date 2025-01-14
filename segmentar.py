# -*- coding: utf-8 -*-
# from __future__ import print_function
import cv2 as cv
import numpy as np
import os
import matplotlib.pyplot as plt
from enum import Enum

MIN_MATCHES = 15 # minimum number of matches to the query to consider it good
SIFT_MATCH_THRESHOLD = 0.75 # threshold to consider matching between keypoints
DATA_DIR = os.path.join(os.getcwd(), "data") # data directory with the query image
QUERY_IMG = 'query_bvb.jpg' # query image of the target to recon by the robot
QUERY_PATH = os.path.join(DATA_DIR, QUERY_IMG)

#class Recon: #Recognize and segment
class Recon:
    def __init__(self, color, myWebcam=False, tb2 = True):
        self.connected = False
        self.webcam = myWebcam
        self.color = color 
        if myWebcam: # for testing purposes based on camera without turtlebot
            self.cam = cv.VideoCapture(0) # camera input
            self.open = self.cam.isOpened()
            if self.open:
                self.connected = True
        self.frame = None
        self.singleImg = False # to process only an image
        self.tb2 = tb2 # to use with tb2 real
        self.query_threshold = MIN_MATCHES # number of keypoints required to assert a match

    def resize(self, new_size): # resize the frame using bicubic interpolation
        return cv.resize(self.frame, new_size, interpolation= cv.INTER_CUBIC)

    def setFrame(self, frame): # set the frame while resizing by interpolation
        self.frame = frame

    # color-based segmentation: https://omes-va.com/deteccion-de-colores2/
    def detect_color(self, frame):
        # gaussian blur to reduce noise
        blur = cv.GaussianBlur(frame, (3,3), 0)
        # convert to HSV to segment object
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)
        # define the range to detect and create a mask
        binary_mask, res = None, None
        if self.color == 'red':
            lower_red_1 = np.array([0,100,20], np.uint8)
            upper_red_1 = np.array([5,255,255], np.uint8)
            lower_red_2 = np.array([175,100,20], np.uint8)
            upper_red_2 = np.array([179,255,255], np.uint8)
            # 2 masks to detect red (light and dark)
            mask1 = cv.inRange(hsv, lower_red_1, upper_red_1)
            mask2 = cv.inRange(hsv, lower_red_2, upper_red_2)
            # binary_mask using threshold to detect red
            binary_mask = cv.add(mask1, mask2)  
        
        elif self.color == 'yellow':
            lower_yellow = np.array([15, 100, 20], np.uint8)
            upper_yellow = np.array([45, 255, 255], np.uint8)
            binary_mask = cv.inRange(hsv, lower_yellow, upper_yellow)
        
        elif self.color == 'green':
            lower_g = np.array([40, 100, 20], np.uint8)
            upper_g = np.array([90, 255, 255], np.uint8)
            binary_mask = cv.inRange(hsv, lower_g, upper_g)
        else:
            print("Error in color attribute. Try 'red' | 'yellow' | 'green' ")
            return None, None, None
        # apply erosion to reduce false positives
        erosion = None
        erosion_type = cv.MORPH_ELLIPSE
        size = (9,9)
        # structuring element
        element = cv.getStructuringElement(erosion_type, size)
        # mask with erosion applied
        erosion = cv.erode(binary_mask, element)

        # mask applied to real RGB frame
        if binary_mask is not None:
            # bitwise-AND mask 
            res = cv.bitwise_and(frame, frame, mask= binary_mask)
        
        return binary_mask, erosion, res

    # returns the largest bounding box according to the image segmented
    def getBoundingBox(self, img, contours):
        max_area = 0 # heuristic largest mask segmented
        most_kp = 0 # heuristic most keypoints from SIFT 
        pos = (0,0) # position of the bb (x,y)
        size = (0,0) # size of the bb (x+w, y+h)
        # find the largest bounding box -> detect the person wearing yellow clothing
        for c in contours:
            x, y, w, h = cv.boundingRect(c)
            # heuristics
            """
            # get the largest bbox due to mask segmented
            if w * h > max_area: 
                pos  = (x,y)
                size = (x + w, y + h)
                max_area = w * h
            """ 
            # check if most of the keypoints detected are in the bbox
            xmin, ymin = x, y
            xmax, ymax = x + w, y + h
            counter = 0 # counter of the keypoints inside the bbox 
            for kp in self.keyp_list: # keyp_list is a list of tuples (x, y)
                kp_x, kp_y = kp
                if kp_x > xmin and kp_x < xmax and kp_y > ymin and kp_y < ymax:
                    counter += 1
            if counter > most_kp:
                pos  = (x,y)
                size = (x + w, y + h)
                most_kp = counter # added
        # draw the bounding box on the img
        cv.rectangle(img, pos, size, (0,0,255), 2)
        # bbox info
        bbox_info = np.array([pos, size])
        return img, bbox_info

    # draws the bounding box of the greatest contour detected
    def draw_bounding_box(self, frame, mask, mode=cv.RETR_EXTERNAL):
        # get the contours
        if self.webcam or self.singleImg or self.tb2:
            contours, _ = cv.findContours(mask, mode, cv.CHAIN_APPROX_SIMPLE)
        else:
            _ , contours, _ = cv.findContours(mask, mode, cv.CHAIN_APPROX_SIMPLE)
        # draw the contours in a copy of the frame
        img_contoured = frame.copy()
        img_array = np.asarray(img_contoured)
        try:
            # draw in blue the contours
            #cv.drawContours(img_array, contours, -1, (255,0,0), 2) # -1 dibuja todos los contornos, el tercer parametro es el color de los contornos en RGB y el ultimo la trasparencia
            # get the bounding box 
            bbox, bbox_info = self.getBoundingBox(img_array, contours)
        except Exception as e:
            #print("No se detectaron contornos")
            return None,None
        return img_array, bbox_info

    # reads from the WebCam and outputs the live feed + detection feed
    def read_from_webcam(self, sift=True, query_path=None): 
        if not self.connected:
            print("Error: La camara no esta conectada")
            return
        print("Reading camera...")
        while(self.open):
            ret, frame = self.cam.read()
            if not ret:
                quit()
            # show height and width of the img: mi webcam {480, 640, 3}
            #print(f"Height, width, channels: {frame.shape}")
            self.setFrame(frame)
            # segmentation based on color
            mask, erosion, segmented = self.detect_color(frame)

            # try matching query and current frame
            match = None # image of the keypoints detected in current frame and query (target)
            if sift:
                match = self.matchingSIFT(query_path) # returns image with the query and current frame matching keypoints

            # draw bounding box of the largest contour detected around the keypoints given by SHIFT
            bbox, _ = self.draw_bounding_box(frame, erosion)

            try:
                # show the current video frames
                cv.imshow('Erosion applied to mask', erosion)
                #cv.imshow('Mask image', mask)            
                cv.imshow('Bounding box detectado', bbox)
                if match is not None:
                    cv.imshow('Query matching', match)
            
            except Exception as e:
                self.open = False
                print("Error al mostrar mascara | imagen segmentada | matching query", e)  
            # exit using key
            if cv.waitKey(25) & 0xFF == ord('q'):
                self.open = False
       
        self.cam.release()
        cv.destroyAllWindows()
        print("All windows closed")

    """
    WAFFLE camera read method
    """
    # read the onboard waffle camera
    def read_from_wafflecam(self, sift=True, query_path=QUERY_PATH): 
        if self.frame is None:
            print("Error: La camara no esta conectada")
            return
        #print("Processing frame...")  
        # show height and width of the img: mi webcam {480, 640, 3}
        #print(f"Height, width, channels: {frame.shape}")
            
        # segmentation based on color
        mask, erosion, segmented = self.detect_color(self.frame)
        # try matching query and current frame
        match = None # image of the keypoints detected in current frame and query (target)
        if sift:
            match = self.matchingSIFT(query_path) # returns image with the query and current frame matching keypoints
        # draw bounding box of the largest contour detected (if there is no detection returns none, none)
        bbox, bbox_info = self.draw_bounding_box(self.frame, erosion)

        try:
            # show the current video frames
            #cv.imshow('Erosion applied to mask', erosion)
            #cv.imshow('Mask image', mask)              
            cv.imshow('Bounding box detectado', bbox)
            if match is not None:
                cv.imshow('Query matching', match)

        except Exception as e:
            print("Error al mostrar imagen con bounding box: ", e)
            pass

        # exit using key
        if cv.waitKey(25) & 0xFF == ord('q'):
            cv.destroyAllWindows()
            print("All windows closed")

        # bbox_info is a numpy array of [pos=(x,y), size=(x+w,y+h) ]
        return bbox_info

    def process_single_img(self, img_path, singleImg=True):
        img = cv.imread(img_path)
        self.setFrame(img)
        self.singleImg = singleImg
        # segmentation based on color
        mask, erosion, segmented = self.detect_color(self.frame)
        # draw bounding box of the largest contour detected (if there is no detection returns none, >
        bbox, bbox_info = self.draw_bounding_box(self.frame, erosion)

        try:
            # show the current video frames
            cv.imshow('Erosion applied to mask', erosion)
            cv.imshow('Mask image', mask)
            cv.imshow('Bounding box detectado', bbox)
            cv.waitKey(0)
            cv.destroyAllWindows()

        except Exception as e:
            print("Error al mostrar mascara y/o imagen segmentada: ", e)
            pass


    def matchingSIFT(self, query_path, img_path=None):
        # frame copy
        frame = None
        # if there is img path check the image given
        if img_path is not None:
            self.setFrame(cv.imread(img_path, cv.IMREAD_GRAYSCALE)) # current frame in grayscale
            frame = cv.imread(img_path) # copy of the current frame in RGB
            frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            # resize both frames to  (820, 620)
            new_size =  (820, 620)
            self.frame = self.resize(new_size)
            frame = self.resize(new_size)

        else: # reading from a camera
            frame = self.frame
            self.frame = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY) # frame to grayscale for SIFT
            #frame = cv.cvtColor(self.frame, cv.COLOR_BGR2RGB) # copy of the crurent frame to show keypoints
            
        try:
            # load query img
            query = cv.imread(query_path) # query original
            #query = cv.cvtColor(query, cv.COLOR_BGR2RGB)
            query_gray = cv.imread(query_path, cv.IMREAD_GRAYSCALE) # query grayscale for SIFT
        except Exception as e:
            print("Error: Query image path wrong: ", e)
            return None

        if query is None or self.frame is None:
            print("Error loading image of the current frame | Error loading image of the query")
            return None

        # create a SIFT descriptor
        #hessianThreshold = 800 # entre 300 y 500 para regular numeros de keypoints detectados
        sift = cv.SIFT_create()
        # detect keypoints and calc descriptors
        train_keyp, train_desc = sift.detectAndCompute(query_gray, None) # nuestro train -> query
        test_keyp, test_desc = sift.detectAndCompute(self.frame, None) # nuestro test -> frame actual
        # do matching using FLANN and 2 neighbours
        neighbours = 2
        #FLANN parameters
        index_params = dict(algorithm = 1, tress = 5)
        search_params = dict(checks = 50)

        matcher = cv.FlannBasedMatcher(index_params, search_params)

        matches = matcher.knnMatch(train_desc, test_desc, k=neighbours)
        # guardar los matches que cumplan cierta distancia
        good = list()
        umbral = SIFT_MATCH_THRESHOLD # distancia umbral entre keyp para considerar buen match
        self.keyp_list = [] # list of keypoints coordinates in the current frame
        for m,n in matches:
            if m.distance < umbral* n.distance:
                good.append(m)

        if len(good) > self.query_threshold:
            print("Min number of matches is satisfied")
            # save keyp_list with the tuple coords of the keypoints to draw the bbox
            for match in good:
                try:
                    x, y = test_keyp[match.trainIdx].pt # coordinates of the keypoint in the current frame
                    self.keyp_list.append( (x, y) )
                except Exception as e:
                    print("Error getting keypoint coordinates: ", e)
                    continue
        # Dibujamos el resultado
        draw_params = dict(matchColor = (0,255,0), singlePointColor = (255,0,0))

        imageMatches = cv.drawMatches(query, train_keyp, frame, test_keyp, good, None, **draw_params)

        return imageMatches
       
        #plt.title('Matching entre query y frame actual usando descriptor SIFT')
        #plt.imshow(imageMatches)
        #plt.show()
        

"""    
import os
import argparse
# ZONA DE PRUEBAS de SEGMENTACION
if __name__ == "__main__":
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
        print("Color no soportado. Usa 'red' o 'yellow'.")
    else:
        print(f"Realizando seguimiento al color: {track_color}")

    recon = Recon(track_color, myWebcam=True)
    DATA_DIR = os.path.join(os.getcwd(), "data")
    # imagenes del supuesto frame actual y el objetivo
    img = 'pasillo3.jpg'
    query = 'query_bvb.jpg'
    img_path = os.path.join(DATA_DIR, img)
    query_path = os.path.join(DATA_DIR, query)
    # reconocer matches mediante SITF (SURF esta pantentado pero es mas eficiente computacionalmente)
    #recon.matchingSIFT(img_path, query_path)
    recon.read_from_webcam(True, query_path)
    
    
    # procesar imagen de un directorio
    #recon.process_single_img(os.path.join(DATA_DIR, img_name))
    
"""
