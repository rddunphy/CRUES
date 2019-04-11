#!/usr/bin/env python
import cv2
import numpy as np

try:
    import RPi.GPIO as GPIO
    from crues_sensors.msg import Vision
    from std_msgs.msg import Bool
    from imutils.video import VideoStream
    import rospy
    pi = True
except ImportError:
    import GPIO_MOCK as GPIO
    pi = False


class RobotDetector:
    def __init__(self):
        self.name = rospy.get_param('hostname', 'robot')
        self.robots = [r for r in rospy.get_param('robots', []) if r['name'] != self.name]
        self.goals = rospy.get_param('goals', [])
        if pi:
            frame_size = (640, 480)
            self.frame_rate = rospy.get_param('~framerate', 10)
            self.cap = VideoStream(src=0, usePiCamera=pi, resolution=frame_size,
                                   framerate=self.frame_rate).start()
            rospy.init_node("vision", anonymous=False)
            self.recording = rospy.get_param("~recording", False)
            self.robot_pub = rospy.Publisher('robots_detected', Vision, queue_size=10)
            self.goal_pub = rospy.Publisher('goal_detected', Bool, queue_size=10)
            self.rate = rospy.Rate(self.frame_rate)
            if self.recording:
                # Define the codec and create VideoWriter object
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.recorder = cv2.VideoWriter('/home/crues/rosbag/output.avi', fourcc, self.frame_rate, (640, 480))
        else:
            self.cap = cv2.VideoCapture(1)

    def search(self, search_frame, objects):
        """Search search_frame for objects and return relevant information
        @:returns found - list of objects found
        @:returns coords - list of (x, y) tuples of centre of mass of objects
        @:returns outlines - list of object outlines
        """
        found, coords, outlines = [], [], []
        for o in objects:
            colour_mask = self.get_colour_mask(search_frame, o['hsv_min'], o['hsv_max'])
            _, contours, _ = cv2.findContours(colour_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            maxsize = 0
            cx, cy, outline = None, None, None
            obj_found = False
            for c in contours:
                if cv2.contourArea(c) > max((10000, maxsize)):  # & cv2.arcLength(c,True):
                    maxsize = cv2.contourArea(c)
                    cx, cy = self.get_centre_point(c)
                    outline = self.get_outline(c)
                    obj_found = True
            if obj_found:
                found.append(o)
                coords.append((cx, cy))
                outlines.append(outline)
        return found, coords, outlines

    def get_colour_mask(self, frame, lower_hsv_bound, higher_hsv_bound):
        """Return a Boolean mask the size of frame, where 1's
        are where 'frames' pixel is within the colour range"""
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if lower_hsv_bound[0] > higher_hsv_bound[0]:
            mask1 = cv2.inRange(hsv_frame, np.array([0, lower_hsv_bound[1], lower_hsv_bound[2]]),
                                np.array(higher_hsv_bound))
            mask2 = cv2.inRange(hsv_frame, np.array(lower_hsv_bound),
                                np.array([179, higher_hsv_bound[1], higher_hsv_bound[2]]))
            mask = mask1 | mask2
        else:
            lower = np.array(lower_hsv_bound)
            upper = np.array(higher_hsv_bound)
            mask = cv2.inRange(hsv_frame, lower, upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def get_outline(self, contour):
        """return the approximate polygon surrounding the contour"""
        epsilon = 0.01 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        return approx

    def get_centre_point(self, contour):
        """find centre of mass of contour"""
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
        return cX, cY

    def spin(self):
        try:
            while not rospy.is_shutdown():
                self._tick()
                self.rate.sleep()
        finally:
            if self.recording:
                self.recorder.release()

    def _draw_bounding_rects(self, frame, objects, outlines):
        for i, o in enumerate(objects):
            x, y, w, h = cv2.boundingRect(outlines[i])
            cv2.rectangle(frame, (x, y), (x + w, y + h), tuple(o['rgb']), 2)
            cv2.putText(frame, o['name'], (x - 20, y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, tuple(o['rgb']), 2)
        self.recorder.write(frame)

    def _tick(self):
        if pi:
            frame = self.cap.read()
            frame = cv2.flip(frame, -1)
        else:
            _, frame = self.cap.read()
        if frame is not None:
            robots, _, r_outlines = self.search(frame, self.robots)
            goals, _, g_outlines = self.search(frame, self.goals)
            msg = Vision()
            msg.robot_list = ", ".join([robot['name'] for robot in robots])
            self.robot_pub.publish(msg)
            self.goal_pub.publish(True if goals else False)
            if self.recording:
                self._draw_bounding_rects(frame, robots + goals, r_outlines + g_outlines)


def _test():
    rd = RobotDetector()
    if pi:
        cap = rd.cap
    else:
        cap = cv2.VideoCapture(0)
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

    rec = False
    while True:
        if pi:
            frame = cap.read()
        else:
            _, frame = cap.read()
        names, found, coords, outlines, highlight_colours = rd.search(frame)

        for i, name in enumerate(names):
            if found[i]:
                x, y, w, h = cv2.boundingRect(outlines[i])
                cv2.rectangle(frame, (x, y), (x + w, y + h), highlight_colours[i], 2)
                cv2.putText(frame, name, (x - 20, y - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, highlight_colours[i], 2)
        cv2.imshow('frame2', frame)
        if not pi:
            out.write(frame)
            key = cv2.waitKey(10)
            if key & 0xFF == ord('r'):
                if rec:
                    out.release()
                    rec = False
                else:
                    print("Recording")
                    rec = True
            if key & 0xFF == ord('q'):
                break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    if pi:
        try:
            cv = RobotDetector()
            cv.spin()
        except rospy.ROSInterruptException:
            cv.cap.stop()
    else:
        _test()
