import cv2
import numpy as np


def main():
	DIM = (640, 480)
	K = np.array([[333.71994064649397, 0.0, 328.90247793390785], [0.0, 332.3864141021737, 221.06175468227764], [0.0, 0.0, 1.0]])
	D = np.array([[-0.022627097023142008], [-0.045919790866641955], [0.055605531135859636], [-0.02114945903451172]])
	video_capture = cv2.VideoCapture(0)
	while (True):
			try:
				_, frame = video_capture.read()
				
				map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
				undistorted_img = cv2.remap(frame, map1, map2, interpolation = cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
	
				cv2.imshow("undistorted image",undistorted_img)
				cv2.imshow("fisheye image",frame)

				cv2.waitKey(1)
			except KeyboardInterrupt:
				break
	video_capture.release()



if __name__=="__main__":
	main()