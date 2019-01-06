#DO NOT OR FILTS TOGETHER
#INSTEAD, JUST FIND CONTOURS ON EACH SEPARATE FILT
#HOW TO AUTOMATICALLY FIND THE BACKGROUND COLORS OF AN IMAGE?
# 1. Given a range of colors defined like below (MIN and MAX for inRange)
#    Step through each color, filt img, close/open, find contours
#    If a large contour exists, assume it is a background color
#    Watch out for being close up to walls, etc.
#!/usr/bin/env python
import cv2
import imutils
import numpy as np

def nothing(x):
    pass

TREE_MIN = np.array([0, 0, 0],np.uint8)
TREE_MAX = np.array([20, 255, 255],np.uint8)
rect_w = 5;
rect_h = 50;
noise_se_w = 5;
noise_se_h = 5;
fill_se_w = 5;
fill_se_h = 5;
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
cv2.createTrackbar('Sat Min','control',75,255,nothing)
cv2.createTrackbar('Sat Max','control',230,255,nothing)
cv2.createTrackbar('Val Min','control',105,255,nothing)
cv2.createTrackbar('Val Max','control',255,255,nothing)
cv2.createTrackbar('Noise SE Width','control2',3,99,nothing)
cv2.createTrackbar('Noise SE Height','control2',7,99,nothing)
cv2.createTrackbar('Fill SE Width','control2',3,99,nothing)
cv2.createTrackbar('Fill SE Height','control2',10,99,nothing)
cv2.createTrackbar('Rect Width','control2',0,99,nothing)
cv2.createTrackbar('Rect Height','control2',0,99,nothing)

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

#img_num = [60,81,94,100,144,158,194,999]
img_num = range(0,187)
k = 0
while k < len(img_num):
    if(k<0):k=0
    best_cnt = np.array([0])
    #img_name = 'sync_photos91/image'+str(img_num[k])+'.jpg'
    #img_name = 'testB_'+str(img_num[k])+'.jpg'
    img_name = "/home/karl/wheele_misc/cone_run4_pics/frame{:04d}.jpg".format(k)
    orig = cv2.imread(img_name)
    rows,cols,nc = orig.shape
    #roi = orig[60:rows,0:cols]
    #orig = roi

    hsv = cv2.cvtColor(orig,cv2.COLOR_BGR2HSV)
    while(1):
        orig = cv2.imread(img_name)
        
        tree_filt = cv2.inRange(hsv, TREE_MIN, TREE_MAX)
        cv2.imshow('tree_filt',tree_filt)

        gray = cv2.cvtColor(orig.copy(), cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (25, 25), 0)
        adapt = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                     cv2.THRESH_BINARY,15,0)
        #adapt = cv2.bitwise_not(adapt)
        cv2.imshow('adapt',adapt)

        rect_w = rect_w +(rect_w==0)
        rect_h = rect_h +(rect_h==0)
        noise_se_w = noise_se_w +(noise_se_w==0)
        noise_se_h = noise_se_h +(noise_se_h==0)
        fill_se_w = fill_se_w +(fill_se_w==0)
        fill_se_h = fill_se_h +(fill_se_h==0)
        #Open binary image
        rect_se = cv2.getStructuringElement(cv2.MORPH_RECT,(rect_w,rect_h))
        noise_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(noise_se_w,noise_se_h))
        fill_se = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(fill_se_w,fill_se_h))
        #erosion then dilation, removes noise in background
        opening = cv2.morphologyEx(tree_filt,cv2.MORPH_OPEN,noise_se)
        cv2.imshow('opn_e',opening)
        #4.Closes the Thresholded Image
        #dilation then erosion, fills holes in foreground
        closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE, fill_se)
        cv2.imshow('cls_e',closing)

        open2 = cv2.morphologyEx(closing,cv2.MORPH_OPEN, rect_se)
        cv2.imshow('opn_r',open2)
        
        #thresh = cv2.Canny(frame,100,200)
        #thresh2 = thresh.copy()
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

        #EDGE DETECTION FOR FINDING CONE
        marker = find_marker(orig)
        # draw a bounding box around the image and display it
        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)
        cv2.drawContours(orig, [box], -1, (0, 255, 0), 2)
        #END EDGE DETECTION APPROACH

        # finding centroids of best_cnt and draw a circle there
        if(best_cnt.ndim == 3):
            M = cv2.moments(best_cnt)
            cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            cv2.circle(orig,(cx,cy),5,255,-1)

        vis1 = np.concatenate((orig, hsv), axis=0)
        vis2 = np.concatenate((tree_filt,open2), axis=0)
        vis3 = cv2.cvtColor(vis2,cv2.COLOR_GRAY2RGB)
        vis = np.concatenate((vis1,vis3),axis=1)
        cv2.imshow('vis',vis)
        cv2.imshow('orig',orig)
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
        TREE_MIN = np.array([hue_min, sat_min, val_min],np.uint8)
        TREE_MAX = np.array([hue_max, sat_max, val_max],np.uint8)
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
