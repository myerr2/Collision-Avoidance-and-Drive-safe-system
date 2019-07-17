#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 17 11:53:12 2019

@author: Murali Yerramsetty
"""
# _*_ coding: utf-8 _*_
#------------------------------------ Start of OCR - Code ---------------------------------------------------#
import RPi.GPIO as gpio
import serial
import time
import picamera
import re
import string
import cv2
from PIL import Image
from pytesser import *
#import pytesser
from curses import ascii
import os
import traceback
import sys
import crop
adrport = serial.Serial(
            port="/dev/ttyAMA0",
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=None)
camera = picamera.PiCamera()
vib = 2
alc = 22
gas = 27
buz = 25
rly = 23
global vflag,gflag,aflag
vflag = False
gflag = False
aflag = False
#------------------------------------------------------
def camini():
   camera.resolution = (1024,768)
   camera.framerate = 30
   time.sleep(2)
   camera.shutter_speed = camera.exposure_speed
   camera.exposure_mode = 'off'
   g=camera.awb_gains
   camera.awb_mode = 'off'
   camera.awb_gains = g
   time.sleep(2)
   return
#-----------------------------------------------------
def capture():
    camera.start_preview()
    time.sleep(1)
    camera.capture('source.jpg')
    camera.stop_preview()
    #camera.close()
def preprocess():
    img = cv2.imread('source.jpg')            #---Read colour image
    grey = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) #--- Convert into grayscale
    cv2.imwrite('grey.jpg',grey)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3)) #---Get kernal of cross shaped with size of 3x3 
    dilated = cv2.dilate(grey,kernel,iterations=2)   #---Apply kernel and iterations on threshold for dilation
    cv2.imwrite('dilate.jpg',dilated)
    eroded = cv2.erode(dilated,kernel,iterations=2)   #---Apply kernel and iterations on threshold for erosion
    cv2.imwrite('erode.jpg',eroded)
    smooth = cv2.blur(eroded,(3,3))
    cv2.imwrite('smooth.jpg',smooth)
    #thresh = cv2.adaptiveThreshold(grey,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,4)
    #thresh = cv2.adaptiveThreshold(smooth,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,4)
    #(thresh,im_bw) = cv2.threshold(grey,50,255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    ret,thresh = cv2.threshold(dilated,80,255,cv2.THRESH_BINARY)
    print("return value: {}".format(ret))
    cv2.imwrite('thresh.jpg',thresh)
    return
#------------------------------------------------------
def ocr_do():
   #image_file = 'fonts_test.png'
   txt = ""
   """
   inpath = 'thresh.jpg'
   outpath = 'thresh.crop.jpg'
   print ("waiting for cropping.....")
   crop.process_image(inpath, outpath)
   time.sleep(1)
   """
   image_file = 'thresh.jpg'
   im = Image.open(image_file)
   #text = image_file_to_string(image_file,graceful_errors = True)
   text = image_to_string(im,True)
   #print "======================output====================\n"
   print ('Length of text: {}'.format(len(text)))
   if len(text) > 0:
      #print "======================output===================="
      #print text
      #print "======================output===================="
      #--Seperate Only Alphabets -----------------------------
      #print "Extracting Alphabets [Characters in lower case only] ...\n"
      #text = text.lower()
      txt = re.sub('[^A-Z0-9]','',text)
      txt = ' '.join(txt.split())
      txt = text
      print ('Length of text: {}'.format(len(txt)))
      print (txt)
   return text
#------------------------------------------------------
def scan():
   global vflag,gflag,aflag
   status = False
   if(gpio.input(vib) == False):
      print("Accident detected")
      vflag = True
   else:
      vflag = False
   if(gpio.input(alc) == True):
      print("Alcohol sensor activated")
      aflag = True
   else:
      aflag = False
   if(gpio.input(gas) == True):
      print("Gas leakage detected")
      gflag = True
   else:
      gflag = False
   #------------------------
   if((aflag or gflag or vflag)==True):
      status = True
   else:
      status = False
   return status
#-----------------------------------------------------
def gpsread():
   rdata = ''
   mstr = ''
   cread = ''
   while(adrport.read() !='$'):
      time.sleep(0.001)
   while(cread!='\n'):
        cread = adrport.read()
        mstr += cread
print (mstr)
   return mstr
#-----------------------------------------------------
def displl():
   flag = False
   qry = "GPGGA"
   lat = ''
   lon = ''
   while(flag == False):
      data = gpsread()
      i = data.find(qry)
      if(i==0):
         flag = True
         print("GPGGA string identified...")
         n = data.find('N')
         e = data.find('E')
         #print(n,e)
         lat = data[n-10:n+1]
         lon = data[e-10:e+1]
         lat = re.sub('[^A-Z.0-9]',' ',lat)
         lon = re.sub('[^A-Z.0-9]',' ',lon)
         print("latitude: {} longitude: {}".format(lat,lon))
   return lat,lon
#------------ SMS Sending Loop ---------------------
def smstx(msg,num):
   lat,lon = displl()
   adrport.write('AT+CMGF=1\r\n')
   time.sleep(0.5)        
   adrport.write('AT+CMGS="{}"\r\n'.format(num))
   #print('AT+CMGS="%s"\r\n' %dnum)
   time.sleep(0.5)
   adrport.write("{}.Location is latitude:{}, longitude:{} -Vehicle security system.\r\n".format(msg,lat,lon))
   time.sleep(0.5)
   adrport.write(ascii.ctrl('z'))
   time.sleep(8)   
   return
#-----------------------------------------------------
def signdetect():
   print ("Taking Picture...\n")
   capture()       #--Capture Image with Specified Settings ---------------
   print ("Preprocessing image for OCR...")
   time.sleep(2)
   preprocess()
   print ("Running OCR...\n")
   time.sleep(2)
   lmt = ocr_do()        #--Run OCR Algorithm on Captured Image --------------
   print("lmt value:{}".format(lmt))
   lmt = re.sub('Q','0',lmt)
   lmt = re.sub('S','5',lmt)
   #lmt = re.sub('Q','0',lmt)
   start = lmt.find('L')
   end = lmt.find('K')
   limit = lmt[start+1:end]
   print("limit value:{}".format(limit))
   limit = re.sub('[^0-9]','',limit)
   limit = ''.join(limit.split())
   #limit = "30"
   print("Detected speed limit: {}Km/hr".format(limit))
   if(len(limit)>0):
      if(int(limit)<100):
         gpio.output(buz,True)
         gpio.output(rly,True)
   return
#---------------------------------
def beep(count):
   for i in range(count):
      gpio.output(buz,True)
      time.sleep(0.3)
      gpio.output(buz,False)
      time.sleep(0.3)
   return
#-------------------------------------------Main Program -----------------------------------------
def main():
   global vflag,gflag,aflag
   gpio.setwarnings(False)
   gpio.setmode(gpio.BCM)
   gpio.setup(vib,gpio.IN)
   gpio.setup(alc,gpio.IN)
   gpio.setup(gas,gpio.IN)
   gpio.setup(buz,gpio.OUT,initial=0)
   gpio.setup(rly,gpio.OUT,initial=0)
   #---------------------------------------
   print ("Initilizing Camera...\n")
   #camini()        #--Camera Settings to Capture Best Possible Image --
   time.sleep(5)
   beep(5)
   while (True):
      if(scan()==True):
         print("Sensors are activated....")
         gpio.output(buz,True)
         gpio.output(rly,True)
         if(vflag == True):
            smstx('Accident occurred,please take care','9030305111')
         elif(gflag == True):
            smstx('Gas leakage occurred ,please take care','9030305111')
         elif(aflag == True):
            smstx('Alcohol sensor activated, please take care','9030305111')
      else:
         #print("Sensors are not activated....")
         gpio.output(buz,False)
         gpio.output(rly,False)
      signdetect()
      time.sleep(5)
      gpio.output(buz,False)
      gpio.output(rly,False)
#-----------------------------------------------END ---------------------------------------------
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        gpio.cleanup()      #except KeyboardInterrupt:
 print "-------- Program Terminated -----------"


