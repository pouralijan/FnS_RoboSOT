import cv2
import numpy
import math
from Calibre import Calibre

from SerialCommand import SerialCommand

class Color(object):
    H_MIN = None
    S_MIN = None
    V_MIN = None
    H_MAX = None
    S_MAX = None
    V_MAX = None

    def __init__(self, name):
        self.NAME = name

class Circle(object):
    def __init__(self, x, y, r, degree, distance, color):
        self.pos = (x, y)
        self.r = r 
        self.degree = degree
        self.distance = distance
        self.color = color

class FramFiltrationSystem(object):
    def __init__(self, frame, color:Color):
        super(FramFiltrationSystem, self).__init__()
        self._frame = fram
        self._color = color
        self.kernel = numpy.ones((5, 5), numpy.uint8)

    def filtring():
        self._frame = cv2.flip(self._frame, 1)

        # # converting to HSV
        hsv = cv2.cvtColor(self._frame, cv2.COLOR_BGR2HSV)
        hue, sat, val = cv2.split(hsv)

        hthresh = cv2.inRange(numpy.array(hue), numpy.array(self._color.H_MIN), numpy.array(self._color.H_MAX))
        sthresh = cv2.inRange(numpy.array(sat), numpy.array(self._color.S_MIN), numpy.array(self._color.S_MAX))
        vthresh = cv2.inRange(numpy.array(val), numpy.array(self._color.V_MIN), numpy.array(self._color.V_MAX))

        # AND h s and v
        tracking = cv2.bitwise_and(hthresh, cv2.bitwise_and(sthresh, vthresh))

        # Some morpholigical filtering
        dilation = cv2.dilate(tracking, self.kernel, iterations=1)
        closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, self.kernel)
        closing = cv2.GaussianBlur(closing, (5, 5), 0)

        #if self._test:
        #    cv2.imshow('HueComp', hthresh)
        #    cv2.imshow('SatComp', sthresh)
        #    cv2.imshow('ValComp', vthresh)
        #    cv2.imshow('closing', closing)
        return closing



class BallDetection(object):
    def __init__(self, frame, color):
        super(BallDetection, self).__init__()
        self._frame = frame
        self._color = color

        self._color = Color("RED")
        self._color.H_MIN = 0
        self._color.S_MIN = 197 
        self._color.V_MIN = 214 
        self._color.H_MAX = 179 
        self._color.S_MAX = 218 
        self._color.V_MAX = 255

    def start(self):
        x_coefficient = 1.0
        y_coefficient = 1.0

        height, width, channels = self._frame.shape
        x_reference, y_reference = width / 2, height

        if type(self._color) is list:
            for color = self._color:
                frame_filtration_system = FramFiltrationSystem(self._frame, color) 
                closing = frame_filtration_system.filtring()
        else:
            frame_filtration_system = FramFiltrationSystem(self._frame, self._color) 
            closing = frame_filtration_system.filtring()

        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(closing, cv2.HOUGH_GRADIENT, 2, 120, param1=120, param2=50, minRadius=10,
                                    maxRadius=0)

    def detect_cirecles(self, frame, color):
        # Draw Circles
        if circles is not None:
            for x, y, r in circles[0, :]:
                xD = x - x_reference
                yD = y - y_reference

                xD *= x_coefficient
                yD *= y_coefficient

                radian = math.atan(xD / yD)
                degree = float(radian * 180 / math.pi)

                if degree < 0:
                    degree += 360

                # This is not corrected :( # TODO: Fix me
                distance = math.sqrt(math.pow(xD, 2) + math.pow(yD, 2))

                if r:
                    if self._test:
                        cv2.line(self._frame, (int(x_reference), int(y_reference)), (x, y), (255, 0, 0), 2)
                        cv2.putText(self._frame, "{}".format(int(degree)), (x, int(y - 20)),
                                    cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                    1.5, (0, 0, 255))

                        cv2.putText(self._frame, "{}".format(int(distance)), (x, int(y + 60)),
                                    cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                    1.5, (0, 0, 255))
                        if r < 30:
                            cv2.circle(self._frame, (x, y), r, (0, 255, 0), 2)
                            cv2.circle(self._frame, (x, y), 1, (0, 255, 0), 5)
                        elif r > 35:
                            cv2.circle(self._frame, (x, y), r, (0, 255, 255), 2)
                            cv2.circle(self._frame, (x, y), 1, (0, 255, 255), 5)
                    else:
                        # Remove print and send x, y, r to MicroController ;)
                        serial_command.send("X: {}, Y: {}, R: {}, Degree: {}, Distance: {}".format(x, y, r, degree, distance))
                        print("X: {}, Y: {}, R: {}, Degree: {}, Distance: {}".format(x, y, r, degree, distance))

            cv2.imshow('tracking', self._frame)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q")]:
                break
            if key in [ord("r"), ord("R")]:
                self.reset()
            if key in [ord("c"), ord("C")]:
                c = Calibre(self._frame)
                c.start()
                if c.get_value():
                    self.hmn, self.smn, self.vmn, self.hmx, self.smx, self.vmx = c.get_value()

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()
