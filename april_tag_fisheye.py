import apriltag
import cv2
import numpy as np
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
	return frame, angle 


def main():
	DIM = (640, 480)
	K = np.array([[333.71994064649397, 0.0, 328.90247793390785], [0.0, 332.3864141021737, 221.06175468227764], [0.0, 0.0, 1.0]])
	D = np.array([[-0.022627097023142008], [-0.045919790866641955], [0.055605531135859636], [-0.02114945903451172]])
	video_capture = cv2.VideoCapture(0)
	map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
	detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())

	while (True):
			try:
				_, frame = video_capture.read()
				gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				undistorted_img = cv2.remap(gray_frame, map1, map2, interpolation = cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
				
				result = detector.detect(undistorted_img)
				
				if result:
					frame, angle = draw_tag(frame,result)
					print(angle)
				
				cv2.imshow("april_tags",frame)
				cv2.waitKey(1)

			except KeyboardInterrupt:
				break
	video_capture.release()



if __name__=="__main__":
	main()