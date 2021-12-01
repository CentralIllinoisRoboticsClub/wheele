#!/usr/bin/env python3
# Copyright 2019 coderkarl. Subject to the BSD license.

import cv2
#import imutils
import numpy as np
from matplotlib import pyplot as plt
import time

def nothing(x):
    pass

TREE_MIN = np.array([0, 0, 0],np.uint8)
TREE_MAX = np.array([20, 255, 255],np.uint8)
rect_w = 3;
rect_h = 6;
noise_se_w = 3;
noise_se_h = 7;
fill_se_w = 3;
fill_se_h = 10;

hue_min = 0
hue_max = 12
sat_min = 70
sat_max = 255
val_min = 70
val_max = 255

#Probably faster to just use channel 0 of hsv image
#and apply simple threshold, rather than inRange

#To omit ground/gravel/grass area, look for a uniform speckled distribution
#created by a hue filter with a range of 1-3 (hopefully just 1)
#Check binary results of hue = 8, hue = 9,... hue = 20
#If such a hue filter gives a uniform speckle in lower half of image,
#assume the specled area is not an object
#if a black object sticks out against the speckled, it is a candidate.

# read the frames
#_,frame = cap.read()
cv2.namedWindow('control', cv2.WINDOW_NORMAL)
cv2.namedWindow('control2', cv2.WINDOW_NORMAL)
cv2.createTrackbar('Hue Min','control', 0,255,nothing)
cv2.createTrackbar('Hue Max','control',12,255,nothing)
cv2.createTrackbar('Sat Min','control',70,255,nothing)
cv2.createTrackbar('Sat Max','control',255,255,nothing)
cv2.createTrackbar('Val Min','control',70,255,nothing)
cv2.createTrackbar('Val Max','control',255,255,nothing)
cv2.createTrackbar('Noise SE Width','control2',3,99,nothing)
cv2.createTrackbar('Noise SE Height','control2',7,99,nothing)
cv2.createTrackbar('Fill SE Width','control2',3,99,nothing)
cv2.createTrackbar('Fill SE Height','control2',10,99,nothing)
cv2.createTrackbar('Rect Width','control2',3,99,nothing)
cv2.createTrackbar('Rect Height','control2',6,99,nothing)

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

def find_marker(image):
    # convert the image to grayscale, blur it, and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(gray, 35, 125)
    cv2.imshow('edge',edged)

    # find the contours in the edged image and keep the largest one;
    # we'll assume that this is our piece of paper in the image
    cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    c = max(cnts, key = cv2.contourArea)

    # compute the bounding box of the of the paper region and return it
    return cv2.minAreaRect(c)


plt.ion()
fig = plt.figure(1)
ax1 = fig.add_subplot(311)
ax2 = fig.add_subplot(312)
ax3 = fig.add_subplot(313)
ax1.set_xlim([0,180])
ax1.set_ylim([0,1.0])
ax2.set_xlim([0,255])
ax2.set_ylim([0,1.0])
ax3.set_xlim([0,255])
ax3.set_ylim([0,1.0])
line1, = ax1.plot(0, 0,'o')
line2, = ax2.plot(0, 0,'o')
line3, = ax3.plot(0, 0,'o')

#img_num = [60,81,94,100,144,158,194,999]
img_num = range(0,187)
k = 73
while k < len(img_num):
    if(k<0):k=0
    best_cnt = np.array([0])
    img_name = "/home/karl/cone_run4_pics/c4frame{:04d}.jpg".format(k)
    print(k)
    image_cv = cv2.imread(img_name)
    rows,cols,nc = image_cv.shape
    #roi = orig[60:rows,0:cols]
    #orig = roi

    hsv = cv2.cvtColor(image_cv,cv2.COLOR_BGR2HSV)
    while(1):
        image_cv = cv2.imread(img_name)
        # PROCESS START
        img_h, img_w, img_d = image_cv.shape
                
        CONE_MIN = np.array([hue_min, sat_min, val_min],np.uint8) #75, 86
        CONE_MAX = np.array([hue_max, sat_max, val_max],np.uint8)
        CONE_MIN2 = np.array([180 - hue_max, sat_min, val_min],np.uint8)
        CONE_MAX2 = np.array([180 - hue_min, sat_max, val_max],np.uint8)
        hsv = cv2.cvtColor(image_cv,cv2.COLOR_BGR2HSV)
        hsv_filt1 = cv2.inRange(hsv, CONE_MIN, CONE_MAX)
        hsv_filt2 = cv2.inRange(hsv, CONE_MIN2, CONE_MAX2)
        hsv_filt = cv2.bitwise_or(hsv_filt1, hsv_filt2)
        
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
        #open2 = hsv_filt
        #try:
        #    self.pub_hsv_filt.publish(self.bridge.cv2_to_imgmsg(open2,"mono8"))
        #except CvBridgeError as e:
        #    print(e)
        
        contours, hierarchy = cv2.findContours(open2,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #python 2 vs 3
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
            if area > max_area and cnt_height/cnt_width > 1.0:# and cnt_height < 40 and cnt_width < 30:
                max_area = area
                best_cnt = cnt
                blob_found = True
                best_height = cnt_height

        # finding centroids of best_cnt and draw a circle there
        hue_list = []
        if(blob_found):
            if(best_cnt.ndim == 3):
                M = cv2.moments(best_cnt)
                cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                cv2.circle(image_cv,(cx,cy),5,255,-1)
                (rx,ry,rw,rh) = cv2.boundingRect(best_cnt)
                cx2,cy2 = (rx+rw/2,ry+rh/2)
                cv2.circle(image_cv,(round(cx2),round(cy2)),5,100,-1)
                cv2.rectangle(image_cv, (rx,ry), (rx+rw,ry+rh), (0, 255, 0), 3)
                hsv_roi = hsv[ry:ry+rh, rx:rx+rw]
                
                for pt in best_cnt[:,0]:
                    x = pt[0]
                    y = pt[1]
                    #hue = hsv[x,y,0]
                    #hue_list.append(hue)
        # PROCESS END

        cv2.imshow('cone_detect',image_cv)
        if hsv_roi.shape[0] > 0:
            nPts = rw*rh
            cv2.imshow('roi', hsv_roi)
            #hue_mask = (hsv_roi[:,:,0] > (180 - hue_max) ) or (hsv_roi[:,:,0] < hue_min )
            histogram, bin_edges = np.histogram(hsv_roi[:,:,0], bins=50, range=(0,180) )
            sat_hist, sat_bin_edges = np.histogram(hsv_roi[:,:,1], bins=50, range=(0,255) )
            val_hist, val_bin_edges = np.histogram(hsv_roi[:,:,2], bins=50, range=(0,255) )
            #plt.figure(1)
            #plt.title("Hue Histogram")
            #plt.xlabel("hue value")
            #plt.ylabel("pixels")
            #plt.xlim([0.0, 1.0])  # <- named arguments do not work here

            #plt.plot(bin_edges[0:-1], histogram)  # <- or here
            #plt.show()
            line1.set_xdata(bin_edges[0:-1])
            line1.set_ydata(histogram/nPts)
            
            line2.set_xdata(sat_bin_edges[0:-1])
            line2.set_ydata(sat_hist/nPts)
            
            line3.set_xdata(val_bin_edges[0:-1])
            line3.set_ydata(val_hist/nPts)
 
            # re-drawing the figure
            fig.canvas.draw()
     
            # to flush the GUI events
            time.sleep(0.03)
            fig.canvas.flush_events()
            

        cv2.imshow('hsv_filt',hsv_filt)           
        cv2.imshow('open2',open2)               

        #vis1 = np.concatenate((orig, hsv), axis=0)
        #vis2 = np.concatenate((tree_filt,open2), axis=0)
        #vis3 = cv2.cvtColor(vis2,cv2.COLOR_GRAY2RGB)
        #vis = np.concatenate((vis1,vis3),axis=1)
        #cv2.imshow('vis',vis)
        #cv2.imshow('orig',orig)
        hue_min = cv2.getTrackbarPos('Hue Min','control')
        hue_max = cv2.getTrackbarPos('Hue Max','control')
        sat_min = cv2.getTrackbarPos('Sat Min','control')
        sat_max = cv2.getTrackbarPos('Sat Max','control')
        val_min = cv2.getTrackbarPos('Val Min','control')
        val_max = cv2.getTrackbarPos('Val Max','control')
        noise_se_w = cv2.getTrackbarPos('Noise SE Width','control2')
        noise_se_h = cv2.getTrackbarPos('Noise SE Height','control2')
        fill_se_w = cv2.getTrackbarPos('Fill SE Width','control2')
        fill_se_h = cv2.getTrackbarPos('Fill SE Height','control2')
        rect_w = cv2.getTrackbarPos('Rect Width','control2')
        rect_h = cv2.getTrackbarPos('Rect Height','control2')
        CONE_MIN = np.array([hue_min, sat_min, val_min],np.uint8)
        CONE_MAX = np.array([hue_max, sat_max, val_max],np.uint8)
        key = cv2.waitKey(33)
        if key == ord('n'):
            k = k+1
            break
        elif key == ord('p'):
            k = k-1
            break
        elif key == 27:
            k = len(img_num)
            break
    

# Clean up everything before leaving
cv2.destroyAllWindows()
#cap.release()
