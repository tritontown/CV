import cv2
import numpy as np
import picamera
import time
import math
import Adafruit_PCA9685
import curses

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
#pwm.set_pwm(2,0,375)

def detect_edges(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #lower_red = np.array([0, 40, 40])
    #upper_red = np.array([6,255,255])

    #lower_yellow = np.array([20, 100, 100], dtype = "uint8")
   # upper_yellow = np.array([30, 255, 255], dtype="uint8")
   # mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_white = cv2.inRange(gray_image, 200, 255)
    #mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
    #mask = cv2.bitwise_and(gray_image, mask_yw)
    
    #mask = cv2.inRange(hsv,lower_red, upper_red)
    edges = cv2.Canny(mask_white, 200, 400)
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                    np.array([]), minLineLength=8, maxLineGap=4)
    return line_segments

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        #logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    
    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height
    y2 = int(y1 * 1 / 4)

    x1 = max(-width, min(2*width, int((y1-intercept)/slope)))
    x2 = max(-width, min(2*width, int((y2-intercept)/slope)))
    return [[x1, y1, x2, y2]]

def detect_lane(frame):
    
    edges = detect_edges(frame)
    cropped_edges = region_of_interest(edges)
    line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(frame, line_segments)
    
    return lane_lines

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    #print(math.tan(steering_angle_radian))
    test = math.tan(steering_angle_radian)
    if (test == 0):
        test = 0.01
    #print(test)
    x2 = int(x1 - height / 2 / test)
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    
    return steering_angle

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=10):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        #pwm.set_pwm(2,0,375)
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        #pwm.set_pwm(2,0,374)
        max_angle_deviation = max_angle_deviation_one_lane
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    
    return stabilized_steering_angle

def main():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(3, 320)
    video_capture.set(4, 240)
    curr_angle = 90
    #pwm.set_pwm(2,0,374)

    #char = screen.getch()
    while (True):
        _, frame = video_capture.read()
        lane_line = detect_lane(frame)
        if lane_line:
            lane_lines_image = display_lines(frame, lane_line)
        else:
            lane_lines_image = frame
        cv2.imshow("Road with Lane line", lane_lines_image)
        lane_lines = detect_lane(frame)
        new_steering_angle = compute_steering_angle(frame, lane_lines)
        curr_angle = stabilize_steering_angle(curr_angle, new_steering_angle, len(lane_lines))
        pwmang = (curr_angle+59)*2
        if (pwmang > 600):
            pwmang = 600
        elif (pwmang < 170):
            pwmang = 170
        print(curr_angle)
        pwm.set_pwm(1,0,pwmang)
        curr_head = display_heading_line(frame, curr_angle)
        cv2.imshow("heading", curr_head)
        cv2.waitKey(1)
        
        
if __name__ == "__main__":
    main()
