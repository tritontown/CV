import apriltag
import cv2
import numpy as np
import pickle
import picamera
import time


video_capture = cv2.VideoCapture(0)
video_capture.set(3, 640)
video_capture.set(4, 480)

detector = apriltag.Detector()
while (True):
    try:
        _, frame = video_capture.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = detector.detect(frame)
        
        if result:
            print(result)
            tf = result[0].tag_family
            cx =  result[0].center[0]
            cy =  result[0].center[1]
            #xyDict = {"CX": cx,"CY": cy}
            #xyArray = np.array([cx,cy])
            #np.save('/home/pi/TritonTown/MQTT/xyArray.npy', xyArray)
            #with open('/home/pi/TritonTown/MQTT/xyDict.pickle', 'wb') as handle:
               #pickle.dump(xyDict, handle, protocol=pickle.HIGHEST_PROTOCOL)
            #print("ourxyArray = ", xyArray)
            #print(tf)
            #print(cx,cy)
        cv2.imshow("apriltags",frame)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        break
video_capture.release
