import apriltag
import cv2
import numpy as np
import pickle
import picamera
import time
from math import atan2,degrees

def draw_tag(frame,result):
	angle = []
	for i in range(len(result)):
		cv2.circle(frame,tuple(result[i].center.astype(int)),5,(0,0,255),3)
		cv2.circle(frame,tuple(result[i].corners[0].astype(int)),5,(0,0,255),3)
		cv2.circle(frame,tuple(result[i].corners[1].astype(int)),5,(0,255,0),3)
		cv2.circle(frame,tuple(result[i].corners[2].astype(int)),5,(255,0,0),3)
		cv2.circle(frame,tuple(result[i].corners[3].astype(int)),5,(0,0,0),3)
		mid = [np.int((result[i].corners[0][0]+result[i].corners[1][0])/2),np.int((result[i].corners[0][1]+result[i].corners[1][1])/2)]
		cv2.line(frame,tuple(result[i].center.astype(int)),tuple(mid),(0,0,255),5)
		angle.append(degrees(atan2(result[i].corners[1][1]-result[i].corners[0][1],result[i].corners[1][0]-result[i].corners[0][0])))
		#cv2.putText(frame,"angle: "+str(np.int(angle)),tuple(center.astype(int)),cv2.FONT_HERSHEY_SIMPLEX,2,128)
	return frame, angle 
	
def main():
	video_capture = cv2.VideoCapture(0)
	#video_capture.set(3, 640)
	#video_capture.set(4, 480)
	
	detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
	while (True):
		try:
			_, frame = video_capture.read()
			gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			result = detector.detect(gray_frame)
			
			if result:
				frame, angle = draw_tag(frame,result)
				print(angle)

			cv2.imshow("apriltags",frame)
			cv2.waitKey(1)
		except KeyboardInterrupt:
			break
	video_capture.release

if __name__=="__main__":
	main()