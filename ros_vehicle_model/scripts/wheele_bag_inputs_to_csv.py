#!/usr/bin/env python

#http://docs.ros.org/api/rosbag_storage/html/c++/
import rosbag
from binascii import hexlify
import rospy

def convertCAN(data, nVars, varSize):
  y = []
  b = 0
  for k in range(nVars):
      #8*varSize will not always work. int(xx,8) is invalid. int(xx,24) may be invalid
      #    if we want int(xx,8), just do int(xx,16), so perhaps we can always just do the max # bits? 32?
      y.append( int(hexlify(data[b:b+varSize]),8*varSize) - (256**varSize)/2)
      b = b+varSize
  return y
    
def get_delta_enc(cur, prev):
  delta = cur - prev
  if(delta > 60000):
      delta = -(65535 - delta)
  elif(delta < -60000):
      delta = 65535 + delta
  return delta

## LOAD BAG DATA
#bag = rosbag.Bag('/home/karl/wheele_bags/wheele_add_compass_GPS_freeze_02Feb2018/2018-02-02-17-54-48.bag')
#bag = rosbag.Bag('/home/karl/wheele_bags/wheele_more_speed_03Feb2018/2018-02-03-17-42-00.bag')

## Get bag file(s) from current directory where this is run
#import glob
#files = glob.glob("*.bag")
import fnmatch
import os
files = []
for root, dirnames, filenames in os.walk('.'):
    for filename in fnmatch.filter(filenames, '*.bag'):
        files.append(os.path.join(root, filename))

GPS_TIME_DIFF_THRESH = 0.051

for f in files:
  print f
  bag = rosbag.Bag(f)
  csv_name = f[0:-3]+'csv'
  csv = open(csv_name,'w')
  ## GPS DATA from /fix topic
  gps_time_list = []
  gps_status_list = []
  gps_service_list = []
  gps_lon_list = []
  gps_lat_list = []
  gps_covar0_list = []
  
  for k, (topic, msg, t) in enumerate(bag.read_messages('/fix')):
    if(k==0):
      t0 = t
    gps_time_list.append(t)
    gps_status_list.append(msg.status.status)
    gps_service_list.append(msg.status.service)
    gps_lon_list.append(msg.longitude)
    gps_lat_list.append(msg.latitude)
    gps_covar0_list.append(msg.position_covariance[0])
    
  ## ALL DATA, MASTER TIME FROM ENCODER DATA
  #print 'bagTime,','frameTime,','delta_left_enc,','delta_right_enc,','gyroz_degx100,', 'gps_time,','gps_status,','gps_service,','lon,','lat,','gps_covar0,','gps_time_diff'
  csv.write('bagTime,frameTime,delta_left_enc,delta_right_enc,gyroz_degx100,gps_time,gps_status,gps_service,lon,lat,gps_covar0,gps_time_diff,\
             compass_time,compass_status,compass_heading_degx100,magX_uTx100,magY_uTx100,magZ_uTx100\n')
  debug_count = 0
  enc_initialized_flag = False
  compass_status = -1
  compass_time = -1
  heading_degx100, magX_uTx100, magY_uTx100, magZ_uTx100 = (0,0,0,0)
  for k, (topic, msg, t) in enumerate(bag.read_messages('/received_messages')):
    if(msg.id == 0x131): #GYRO
      gyroz_list = convertCAN(msg.data,1,2)
      gyroz_degx100 = gyroz_list[0]
    elif(msg.id == 0x133): #COMPASS
      heading_degx100, magX_uTx100, magY_uTx100, magZ_uTx100 = convertCAN(msg.data,4,2)
      compass_time_raw = t
      compass_status = 1
    elif(msg.id == 0x105): #ENCODERS
      left_enc, right_enc = convertCAN(msg.data,2,2)
      if(not enc_initialized_flag):
        prev_left_enc = left_enc
        prev_right_enc = right_enc
        enc_initialized_flag = True
      delta_left_enc = get_delta_enc(left_enc, prev_left_enc)
      delta_right_enc = get_delta_enc(right_enc, prev_right_enc)
      
      if(len(gps_time_list) > 0):
        gps_time_diff = (gps_time_list[0] - t).to_sec()
      else:
        gps_time_diff = 100.0
        
      found_gps_match = False
      while(not found_gps_match and gps_time_diff < GPS_TIME_DIFF_THRESH):
        gps_time_raw = gps_time_list.pop(0)
        gps_time_diff = (gps_time_raw - t).to_sec()
        if(abs(gps_time_diff) < GPS_TIME_DIFF_THRESH):
          gps_time = (gps_time_raw-t0).to_sec()
          gps_status = gps_status_list.pop(0)
          gps_service = gps_service_list.pop(0)
          gps_lon = gps_lon_list.pop(0)
          gps_lat = gps_lat_list.pop(0)
          gps_covar0 = gps_covar0_list.pop(0)
          found_gps_match = True
        
      if(not found_gps_match):        
        gps_time = -1
        gps_status = -9
        gps_service = -9
        gps_lon = 0
        gps_lat = 0
        gps_covar0 = 0
       
      if(compass_status == 1):
        compass_time = (compass_time_raw-t0).to_sec()
      else:
        compass_time = -1
      
      #print t-t0, ',', msg.header.stamp-t0, ',', delta_left_enc, ',', delta_right_enc, ',', gyroz_degx100, ',', gps_time, ',', gps_status, ',', gps_service, ',', gps_lon, ',', gps_lat, ',', gps_covar0, ',', gps_time_diff
      csv.write('%.9f,%.9f,%d,%d,%d,%.9f,%d,%d,%.10f,%.10f,%.5f,%.9f,\
                 %.9f,%d,%d,%d,%d,%d\n'
       % ((t-t0).to_sec(), (msg.header.stamp-t0).to_sec(), delta_left_enc, delta_right_enc, gyroz_degx100,
          gps_time, gps_status, gps_service, gps_lon, gps_lat, gps_covar0, gps_time_diff,
          compass_time,compass_status,heading_degx100,magX_uTx100,magY_uTx100,magZ_uTx100 ))
      prev_left_enc = left_enc
      prev_right_enc = right_enc
      compass_status = -1

  bag.close()
  csv.close()


