import cv2
import imutils
from crues_sensors.msg import Vision

import numpy as np
try:
    import RPi.GPIO as GPIO
except ImportError:
    from crues import GPIO_MOCK as GPIO
import rospy

class RobotDetector():
    def __init__(self):
        self.name = "Robot"
        self.robot_colours = [("Inky", ([110, 100, 40], [140, 255, 255]),(255, 0, 0)),
                              ("Clyde", ([20, 120, 100],[50, 255, 255]), (0, 255, 255)),
                              ("Blinky", ([170, 110, 60], [10, 255, 255]), (0, 0, 255))]
        self.cap = cv2.VideoCapture(0)
        self.pub = rospy.Publisher('robots_detected', Vision, queue_size=10)

    def search(self, search_frame):
        ''' Search search_frame for other robots and return relevant information
        @:returns names - list of robot name
        @:returns found - list, true if the object is in the frame
        @:returns coords - list of (x, y) tuples of centre of mass of objects
        @:returns outlines - list of object outlines
        @:returns highlight_colours - for rendering to frame, colour of text and highlighting box
        '''
        names, found, coords, outlines, highlight_colours = [], [], [], [], []
        for (name, colour_bounds, highlight_colour) in self.robot_colours:
            colour_mask = self.get_colour_mask(search_frame, colour_bounds[0], colour_bounds[1])
            contours, _ = cv2.findContours(colour_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            maxsize = 0
            cx, cy, outline = None, None, None
            obj_found = False
            for c in contours:
                if cv2.contourArea(c) > max((300, maxsize)):  # & cv2.arcLength(c,True):
                    maxsize = cv2.contourArea(c)
                    cx, cy = self.get_centre_point(c)
                    outline = self.get_outline(c)
                    obj_found = True
            names.append(name)
            found.append(obj_found)
            coords.append((cx, cy))
            outlines.append(outline)
            highlight_colours.append(highlight_colour)
        return names, found, coords, outlines, highlight_colours

    def get_colour_mask(self, frame, lower_hsv_bound, higher_hsv_bound):
        '''Return a Boolean mask the size of frame, where 1's
        are where 'frames' pixel is within the colour range'''
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        if(lower_hsv_bound[0]>higher_hsv_bound[0]):
            mask1 = cv2.inRange(hsv_frame, np.array([0, lower_hsv_bound[1], lower_hsv_bound[2]]), np.array(higher_hsv_bound))
            mask2 = cv2.inRange(hsv_frame, np.array(lower_hsv_bound), np.array([179, higher_hsv_bound[1], higher_hsv_bound[2]]))
            mask = mask1 | mask2
        else:
            lower = np.array(lower_hsv_bound)
            upper = np.array(higher_hsv_bound)
            mask = cv2.inRange(hsv_frame, lower, upper)

        # lower_green = np.array([50, 80, 50])
        # upper_green = np.array([100, 255, 255])
        # mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        #


        # GBMask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def get_outline(self, contour):
        '''return the approximate polygon surrounding the contour'''
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return approx

    def get_centre_point(self, contour):
        '''find centre of mass of contour'''
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        return cX, cY

    def spin(self):
        rate = rospy.Rate(rospy.get_param('~rate', 50))
        try:
            while not rospy.is_shutdown():
                self._tick()
                rate.sleep()
        finally:
            self._cleanup()

    def _tick(self):
        _, frame = self.cap.read()
        names, found, coords, outlines, highlight_colours = self.search(frame)
        #for i, name in enumerate(names):
        #    if found[i]:
                #x, y, w, h = cv2.boundingRect(outlines[i])
                #cv2.rectangle(frame, (x, y), (x + w, y + h), highlight_colours[i], 2)
                #cv2.putText(frame, name, (x - 20, y - 20),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, highlight_colours[i], 2)
        msg = Vision()
        msg.robot_list  = ",".join(names)
        self.pub.publish(msg)

#if __name__ == '__main__':
#
#    cap = cv2.VideoCapture(0)
#    rd = RobotDetector()
#    while 1:
#        _, frame = cap.read()
#        names, found, coords, outlines, highlight_colours = rd.search(frame)
#
#        for i, name in enumerate(names):
#            if found[i]:
#                #cv2.drawContours(frame, [outline], -1, (0, 255, 0), 2)
#                #cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
#                x, y, w, h = cv2.boundingRect(outlines[i])
#                cv2.rectangle(frame, (x, y), (x + w, y + h), highlight_colours[i], 2)
#                cv2.putText(frame, name, (x - 20, y - 20),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, highlight_colours[i], 2)
#        cv2.imshow('frame2', frame)
#        if cv2.waitKey(10) & 0xFF == ord('q'):
#            break
#    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        cv = RobotDetector()
        cv.spin()
    except rospy.ROSInterruptException:
        pass
