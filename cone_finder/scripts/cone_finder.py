#!/usr/bin/env python
# Copyright 2019 coderkarl. Subject to the BSD license.

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import sys
import threading

from dynamic_reconfigure.server import Server
from cone_detector.cfg import ConeConfig

rect_w = 3;
rect_h = 6;
noise_se_w = 3;
noise_se_h = 7;
fill_se_w = 3;
fill_se_h = 10;

only_publish_detections = False

class ConeFinder:
    def __init__(self):        
        self.node_name = "Cone Finder"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/image_raw", Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("cone_img", Image, queue_size=1)
        self.pub_hsv_filt = rospy.Publisher("hsv_filt", Image, queue_size=1)
        self.pub_cone_pose = rospy.Publisher("raw_cone_pose", PoseStamped, queue_size = 5)
        self.bridge = CvBridge()
        
        self.config = None
        self.srv = Server(ConeConfig, self.config_callback)

        rospy.loginfo("Initialized Cone Finder")        

    def config_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {hue_min}, {hue_max}, {sat_min}, {sat_max}, {val_min}, {val_max}, {double_param},\ 
            {str_param}, {bool_param}, {size}""".format(**config))
        self.config = config
        return config

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        
    def detect_shape(self, c):
        # https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.1 * peri, True) #0.04*peri
        
        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"

        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"

        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"

        # return the name of the shape
        return shape

    def processImage(self, image_msg):
        #rospy.loginfo("processImage")
        blob_found = False
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        img_h, img_w, img_d = image_cv.shape
        
        if(not self.config == None):
                
            CONE_MIN = np.array([self.config["hue_min"], self.config["sat_min"], self.config["val_min"]],np.uint8) #75, 86
            CONE_MAX = np.array([self.config["hue_max"], self.config["sat_max"],self.config["val_max"]],np.uint8)
            CONE_MIN2 = np.array([180-self.config["hue_max"], self.config["sat_min"], self.config["val_min"]],np.uint8)
            CONE_MAX2 = np.array([180-self.config["hue_min"], self.config["sat_max"],self.config["val_max"]],np.uint8)
            hsv = cv2.cvtColor(image_cv,cv2.COLOR_BGR2HSV)
            hsv_filt1 = cv2.inRange(hsv, CONE_MIN, CONE_MAX)
            hsv_filt2 = cv2.inRange(hsv, CONE_MIN2, CONE_MAX2)
            hsv_filt = cv2.bitwise_or(hsv_filt1, hsv_filt2)
            
            #Open binary image
            rect_se = cv2.getStructuringElement(cv2.MORPH_RECT,(rect_w,rect_h))
            noise_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(noise_se_w,noise_se_h))
            fill_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(fill_se_w,fill_se_h))
            #erosion then dilation, removes noise in background
            #opening = cv2.morphologyEx(hsv_filt,cv2.MORPH_OPEN,noise_se)
            #4.Closes the Thresholded Image
            #dilation then erosion, fills holes in foreground
            #closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE, fill_se)
            #open2 = cv2.morphologyEx(closing,cv2.MORPH_OPEN, rect_se)
            open2 = hsv_filt
            #try:
            #    self.pub_hsv_filt.publish(self.bridge.cv2_to_imgmsg(open2,"mono8"))
            #except CvBridgeError as e:
            #    print(e)
            
            _, contours, hierarchy = cv2.findContours(open2,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #python 2 vs 3
            # finding contour with maximum area and store it as best_cnt
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                pts = cnt[:,0]
                x = pts[:,0]
                y = pts[:,1]
                cnt_height = max(y)-min(y)
                cnt_width = max(x)-min(x)
                #Longest Distance between 2 points/area
                if area > max_area and cnt_height/cnt_width > 0.5:# and cnt_height < 40 and cnt_width < 30:
                    max_area = area
                    best_cnt = cnt
                    blob_found = True
                    best_height = cnt_height

            # finding centroids of best_cnt and draw a circle there
            if(blob_found):
                if(best_cnt.ndim == 3):
                    M = cv2.moments(best_cnt)
                    cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    cv2.circle(image_cv,(cx,cy),5,255,-1)
                    (rx,ry,rw,rh) = cv2.boundingRect(best_cnt)
                    cx2,cy2 = (rx+rw/2,ry+rh/2)
                    cv2.circle(image_cv,(cx2,cy2),5,100,-1)
                    cv2.rectangle(image_cv, (rx,ry), (rx+rw,ry+rh), (0, 255, 0), 3)
                    #rospy.loginfo("Cone Found at pixel x,y: %d, %d",int(cx),int(cy))
                    
                    px_norm = (cx-img_w/2.0)/float(img_w)
                    py_norm = (cy-img_h/2.0)/float(img_h)
                    ph_norm = best_height/float(img_h)
                    ideal_py_norm = -0.05
                    
                    local_x = 0.5/ph_norm
                    rospy.loginfo("Cone local_x: %0.1f, py_norm: %0.2f",local_x, py_norm)
                    blob_shape = self.detect_shape(best_cnt)
                    print(blob_shape)
                    if(local_x < 10.0 and (abs(py_norm-ideal_py_norm) < 0.2 and blob_shape=="triangle") ): #TODO: parameter
                        local_y = -0.85 * local_x * px_norm
                        cone_pose = PoseStamped()
                        cone_pose.header.frame_id = "base_link"
                        cone_pose.header.stamp = rospy.Time.now() # assign this to rospy.Time.now() at the beginning of the processImage function
                        cone_pose.pose.orientation.w = 1.0
                        cone_pose.pose.position.x = local_x
                        cone_pose.pose.position.y = local_y
                        self.pub_cone_pose.publish(cone_pose)
            
            
            try:
                self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv,"bgr8"))
            except CvBridgeError as e:
                print(e)
        
        #rospy.loginfo("Cone Search Done")
        self.thread_lock.release()


if __name__=="__main__":    
    rospy.init_node('cone_finder')
    cone_finder = ConeFinder()
    rospy.spin()
