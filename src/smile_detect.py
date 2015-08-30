#!/usr/bin/env python
import roslib
roslib.load_manifest('smile_detector')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class smile_detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("smile_detector",Int8)
    rospy.init_node('smile_detector', anonymous=True)
    self.bridge = CvBridge()
    'self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)'
    self.image_sub = rospy.Subscriber("/head_xtion/rgb/image_color",Image,self.callback)
    self.face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    self.smile_cascade = cv2.CascadeClassifier("haarcascade_smile.xml")
  def callback(self,data):
    try:
      cv_image = cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY) 
      'gray = cv2.pyrDown(gray)'
      gray = cv2.pyrDown(gray)
      scale=2
      faces = self.face_cascade.detectMultiScale(gray,1.3,5)
      image=cv_image
      NumberOfSmiles=0
      for (x,y,w,h) in faces:
            cv2.rectangle(image,(x*scale,y*scale),(x*scale+w*scale,y*scale+h*scale),(255,0,0),2)
            roi=gray[y:y+h, x:x+w]
            roic=image[y*scale:y*scale+h*scale,x*scale:x*scale+w*scale]
            smile = self.smile_cascade.detectMultiScale(roi,1.3,5)
            for (x1,y1,w1,h1) in smile:
                cv2.rectangle(roic,(x1*scale,y1*scale),(x1*scale+w1*scale,y1*scale+h1*scale),(0,0,255),2)
            if len(smile)>0:
               NumberOfSmiles=NumberOfSmiles+1
      'cv2.imshow("view",image)'
      'cv2.waitKey(1)'
      'print "%s number of faces"%(NumberOfSmiles)'
    except cv2.error, e:
      print e
    try:
      self.image_pub.publish(NumberOfSmiles)
    except cv2.error, e:
      print e

def main(args):
  ic = smile_detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
