from picamera import PiCamera
from time import sleep
import cv2

video_capture = cv2.VideoCapture(0)
id = 0
while (True):		
	_,frame = video_capture.read()
	cv2.imshow("apriltags",frame)
	k = cv2.waitKey(1)
	if k%256 == 32:
		print("capture")
		cv2.imwrite("result/IMAGE"+str(id)+".jpg",frame)
		id+=1