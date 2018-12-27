#!/usr/bin/env python
import cv2
import numpy as np
import math
import csv
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

class ConeTrainer:
    def __init__(self):
        self.cone_file = open('/home/karl/temp_cone_data.csv',mode='w')
        self.writer = csv.writer(self.cone_file, delimiter=',')
        self.writer.writerow(['img_num','x','y','dist','angle_rad','pix_col_norm','pix_row_norm','pix_height_norm'])
        self.writer.writerow([0, 7.1, 3.1, 7.9, 0.1, 0.3, 0.4, 0.5])
        
        self.img_num = 0
        self.cone_truth_x = 0.0
        self.cone_truth_y = 0.0
        self.cone_truth_dist = 0.0
        self.cone_truth_angle = 0.0
        
        self.node_name = "Cone Trainer"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/image_raw", Image, self.cbImage, queue_size=1)
        self.sub_cone_truth = rospy.Subscriber("base_cone_pose", PoseStamped, self.cone_truth_callback, queue_size=10)
        self.pub_image = rospy.Publisher("cone_img", Image, queue_size=1)
        self.pub_hsv_filt = rospy.Publisher("hsv_filt", Image, queue_size=1)
        self.pub_cone_pose = rospy.Publisher("raw_cone_pose", PoseStamped, queue_size = 5)
        self.bridge = CvBridge()
        
        self.config = None
        self.srv = Server(ConeConfig, self.config_callback)

        rospy.loginfo("Initialized Cone Trainer")
    
    def __del__(self):
        self.cone_file.close()

    def cone_truth_callback(self, data):
        p = data.pose.position
        x = p.x
        y = p.y
        self.cone_truth_x = x
        self.cone_truth_y = y
        self.cone_truth_dist = np.sqrt(x**2 + y**2)
        self.cone_truth_angle = math.atan2(y,x)

    def config_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {hue_min}, {hue_max}, {double_param},\ 
            {str_param}, {bool_param}, {size}""".format(**config))
        self.config = config
        return config

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        #rospy.loginfo("processImage")
        blob_found = False
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        img_h, img_w, img_d = image_cv.shape
        #print 'image cb, img_w, img_h: ', img_w, img_h
        
        if(not self.config == None):
                
            CONE_MIN = np.array([self.config["hue_min"], self.config["sat_min"], self.config["val_min"]],np.uint8) #75, 86
            CONE_MAX = np.array([self.config["hue_max"], self.config["sat_max"],self.config["val_max"]],np.uint8)
            hsv = cv2.cvtColor(image_cv,cv2.COLOR_BGR2HSV)
            hsv_filt = cv2.inRange(hsv, CONE_MIN, CONE_MAX)
            
            #Open binary image
            rect_se = cv2.getStructuringElement(cv2.MORPH_RECT,(rect_w,rect_h))
            noise_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(noise_se_w,noise_se_h))
            fill_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(fill_se_w,fill_se_h))
            #erosion then dilation, removes noise in background
            opening = cv2.morphologyEx(hsv_filt,cv2.MORPH_OPEN,noise_se)
            #4.Closes the Thresholded Image
            #dilation then erosion, fills holes in foreground
            closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE, fill_se)
            open2 = cv2.morphologyEx(closing,cv2.MORPH_OPEN, rect_se)
            
            try:
                self.pub_hsv_filt.publish(self.bridge.cv2_to_imgmsg(open2,"mono8"))
            except CvBridgeError as e:
                print(e)
            
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
                    #rospy.loginfo("Cone Found at pixel x,y: %d, %d",int(cx),int(cy))
                    try:
                        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv,"bgr8"))
                    except CvBridgeError as e:
                        print(e)
                    
                    self.writer.writerow([self.img_num, self.cone_truth_x, self.cone_truth_y, self.cone_truth_dist, self.cone_truth_angle,
                                          (cx-img_w/2.0)/float(img_w), (cy-img_h/2.0)/float(img_h), best_height/float(img_h)])
                    px_norm = (cx-img_w/2.0)/float(img_w)
                    py_norm = (cy-img_h/2.0)/float(img_h)
                    ph_norm = best_height/float(img_h)
                    ideal_py_norm = -0.15
                    
                    local_x = 0.5/ph_norm
                    if(local_x < 6.0 and abs(py_norm-ideal_py_norm) < 0.05):
                        local_y = -0.85 * local_x * px_norm
                        cone_pose = PoseStamped()
                        cone_pose.header.frame_id = "base_link"
                        cone_pose.header.stamp = rospy.Time.now()
                        cone_pose.pose.orientation.w = 1.0
                        cone_pose.pose.position.x = local_x
                        cone_pose.pose.position.y = local_y
                        self.pub_cone_pose.publish(cone_pose)
        
        #rospy.loginfo("Cone Search Done")
        self.thread_lock.release()
        self.img_num += 1


if __name__=="__main__":    
    rospy.init_node('cone_trainer')
    cone_trainer = ConeTrainer()
    rospy.spin()
