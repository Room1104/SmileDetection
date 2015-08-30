#!/usr/bin/env python
import roslib
roslib.load_manifest('smile_detector')
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import Image,UpperBodyDetector
from cv_bridge import CvBridge, CvBridgeError
import message_filter
class smile_detector:

  def __init__(self):
    self.image_pub = rospy.Publisher("smile_detector",Int8)
    rospy.init_node('smile_detector', anonymous=True)
    self.bridge = CvBridge()
    'self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)'
    self.image_sub = message_filter.Subscriber("/head_xtion/rgb/image_color",Image)
    self.frame_sub = message_filter.Subscriber("/upper_body_detector/detections",UpperBodyDetector)
    ts = message_filters.TimeSynchronizer([self.image_sub, self.frame_sub], 10) 
    ts.registerCallback(callback)
    'self.image_sub = rospy.Subscriber("/head_xtion/rgb/image_color",Image,self.callback)'
    self.face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    self.smile_cascade = cv2.CascadeClassifier("haarcascade_smile.xml")
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data.Image, "bgr8")
      frame = data.UpperBodyDetector
      gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY) 
      'gray = cv2.pyrDown(gray)'
      gray = cv2.pyrDown(gray)
      scale=2
      'faces = self.face_cascade.detectMultiScale(gray,1.3,5)'
      image=cv_image
      NumberOfSmiles=0
      for (frame) in faces:
            cv2.rectangle(image,(frame.pos_x,frame.pos_y),(frame.width,frame.height),(255,0,0),2)
            roi=gray[frame.pos_y/scale:frame.pos_y/scale+frame.height/scale, frame.pos_x/scale:frame.pos_x/scale+frame.width/scale]
            roic=image[frame.pos_y:frame.pos_y+frame.height,frame.pos_x:frame.pos_x+frame.pos_x/scale]
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
