import RPi.GPIO as GPIO
import numpy as np
import cv2
import cv2.cv as cv
from picamera.array import PiRGBArray     
from picamera import PiCamera
import time
 
 
GPIO.setmode(GPIO.BOARD)
 
ULTRA_TRIG1 = 29     
ULTRA1_EC = 31
 ULTRA_TRIG2 = 36     
ULTRA2_EC = 37
 ULTRA_TRIG3 = 33     
ULTRA3_EC = 35
 
MOTOR1_LEFT=18 
MOTOR1_RIGHT=22
 MOTOR2_LEFT=21 
MOTOR2_RIGHT=19
 
LED=13 
 
GPIO.setup(ULTRA_TRIG1,GPIO.OUT) 
GPIO.setup(ULTRA1_EC,GPIO.IN)     
GPIO.setup(ULTRA_TRIG2,GPIO.OUT) 
GPIO.setup(ULTRA2_EC,GPIO.IN)
GPIO.setup(ULTRA_TRIG3,GPIO.OUT)
GPIO.setup(ULTRA3_EC,GPIO.IN)
GPIO.setup(LED,GPIO.OUT)
 
GPIO.output(ULTRA_TRIG1, False)
GPIO.output(ULTRA_TRIG2, False)
GPIO.output(ULTRA_TRIG3, False)
 
def sonar(TRIG,ECHO):
      start=0
      stop=0
      GPIO.setup(TRIG,GPIO.OUT) 
      GPIO.setup(ECHO,GPIO.IN)     
    
      GPIO.output(TRIG, False)
    
      time.sleep(0.01)
 
 
      GPIO.output(TRIG, True)
      time.sleep(0.00001)
      GPIO.output(TRIG, False)
      begin = time.time()
      while GPIO.input(ECHO)==0 and time.time()<begin+0.10:
            start = time.time()
    
      while GPIO.input(ECHO)==1 and time.time()<begin+0.1:
            stop = time.time()
    
     
      elapsed = stop-start
 
      distance = elapsed * 34000
    
 
      distance = distance / 2
    
      print "Distance : %.1f" % distance
 
      return distance
 
GPIO.setup(MOTOR1_LEFT, GPIO.OUT)
GPIO.setup(MOTOR1_RIGHT, GPIO.OUT)
GPIO.setup(MOTOR2_LEFT, GPIO.OUT)
GPIO.setup(MOTOR2_RIGHT, GPIO.OUT)
 
def forward():
      GPIO.output(MOTOR1_LEFT, GPIO.HIGH)
      GPIO.output(MOTOR1_RIGHT, GPIO.LOW)
      GPIO.output(MOTOR2_LEFT, GPIO.HIGH)
      GPIO.output(MOTOR2_RIGHT, GPIO.LOW)
    
def reverse():
      GPIO.output(MOTOR1_LEFT, GPIO.LOW)
      GPIO.output(MOTOR1_RIGHT, GPIO.HIGH)
      GPIO.output(MOTOR2_LEFT, GPIO.LOW)
      GPIO.output(MOTOR2_RIGHT, GPIO.HIGH)
    
def turning_Right():
      GPIO.output(MOTOR1_LEFT,GPIO.LOW)
      GPIO.output(MOTOR1_RIGHT,GPIO.HIGH)
      GPIO.output(MOTOR2_LEFT,GPIO.HIGH)
      GPIO.output(MOTOR2_RIGHT,GPIO.LOW)
    
def turning_Left():
      GPIO.output(MOTOR1_LEFT,GPIO.HIGH)
      GPIO.output(MOTOR1_RIGHT,GPIO.LOW)
      GPIO.output(MOTOR2_LEFT,GPIO.LOW)
      GPIO.output(MOTOR2_RIGHT,GPIO.HIGH)
 
def stop():
      GPIO.output(MOTOR1_RIGHT,GPIO.LOW)
      GPIO.output(MOTOR1_LEFT,GPIO.LOW)
      GPIO.output(MOTOR2_RIGHT,GPIO.LOW)
      GPIO.output(MOTOR2_LEFT,GPIO.LOW)
   
 
def location(blob):
    largest_contour=0
    cont_index=0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for index, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
            largest_contour=area
          
            cont_index=index
                             
    r=(0,0,1,1)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
      
    return r,largest_contour

def segment_colour(frame):   
    hsv =  cv2.cvtColor(frame, cv2.cv.CV_BGR2HSV)
    mask_1 = cv2.inRange(hsv, np.array([160, 160,10]), np.array([190,255,255]))
    ycr=cv2.cvtColor(frame,cv2.cv.CV_BGR2YCrCb)
    mask_2=cv2.inRange(ycr, np.array((0.,165.,0.)), np.array((255.,255.,255.)))
#masking both of them to get a better color range
    mask = mask_1 | mask_2
    dilate = np.ones((8,8),np.uint8)
    erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,erode)     
    mask=cv2.dilate(mask,dilate)    
    return mask

#camera Module adjusting parameters 
camera = PiCamera()
rawCapture = PiRGBArray(camera, size=(150, 130))
camera.resolution = (160,130)
camera.framerate = 15
 
time.sleep(0.001)
for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    
      frame = image.array
      frame=cv2.flip(frame,1)
      global x_cen
      global y_cen
      x_cen=0.
      y_cen=0.
      hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      red_mask=segment_colour(frame)     
      locations,area=location(red_mask)
      x,y,w,h=locations
    
      Centre_dist = sonar(ULTRA_TRIG2,ULTRA2_EC)
      Right_dist = sonar(ULTRA_TRIG3,ULTRA3_EC)
      Left_dist = sonar(ULTRA_TRIG1,ULTRA1_EC)
            
      if (w*h) < 10:
            found=0
      else:
            found=1
            ball_rec = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
            x_cen=x+((w)/2)
            y_cen=y+((h)/2)
           
            x_cen-=80
            y_cen=6--y_cen
            print x_cen,y_cen
      initial=400
      f=0
      GPIO.output(LED,GPIO.LOW)         
      if(found==0):
            if f==0:
                  turning_Right()
                  time.sleep(0.05)
            else:
                  turning_Left()
                  time.sleep(0.05)
            stop()
            time.sleep(0.0130)
    
      elif(found==1):
            if(area<initial):
                  if(Centre_dist<10):
                        if Right_dist>=8:
                              turning_Right()
                              time.sleep(0.006)
                              stop()
                              time.sleep(0.0130)
                              forward()
                              time.sleep(0.006)
                              stop()
                              time.sleep(0.0130)
                              turning_Left()
                              time.sleep(0.006)
                        elif Left_dist>=8:
                              turning_Left()
                              time.sleep(0.006)
                              stop()
                              time.sleep(0.0130)
                              forward()
                              time.sleep(0.006)
                              stop()
                              time.sleep(0.0130)
                              turning_Right()
                              time.sleep(0.006)
                              stop()
                              time.sleep(0.0130)
                        else:
                              stop()
                              time.sleep(0.01)
                  else:
             #mOving forwrd
                        forward()
                        time.sleep(0.006)
            elif(area>=initial):
                  initial2=6700
                  if(area<initial2):
                        if(Centre_dist>10):
                              if(x_cen<=-25 or x_cen>=25):
                                    if(x_cen<0):
                                          f=0
                                          turning_Right()
                                          time.sleep(0.030)
                                    elif(x_cen>0):
                                          f=1
                                          turning_Left()
                                          time.sleep(0.030)
                              forward()
                              time.sleep(0.00003)
                              stop()
                              time.sleep(0.006)
                        else:
                              stop()
                              time.sleep(0.01)
 
                  else:
                        GPIO.output(LED,GPIO.HIGH)
                        time.sleep(0.1)
                        stop()
                        time.sleep(0.1)
      rawCapture.truncate(0)
        
      if(cv2.waitKey(1) & 0xff == ord('q')):
            break
 
GPIO.cleanup()

