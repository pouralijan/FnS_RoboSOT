import cv2
import numpy
import math
from Calibre import Calibre

from SerialCommand import SerialCommand


class BallDetection(object):
    def __init__(self, test=False):
        super(BallDetection, self).__init__()
        # True only for test code ;)
        self._test = test
        self.cap = cv2.VideoCapture(0)
        self.kernel = numpy.ones((5, 5), numpy.uint8)

        self.hmn = 0
        self.smn = 197
        self.vmn = 214
        self.hmx = 179
        self.smx = 218
        self.vmx = 255

    def reset(self):
        # Red
        # self.hmn = 0
        # self.smn = 197
        # self.vmn = 214
        # self.hmx = 179
        # self.smx = 218
        # self.vmx = 255
        # min H = 95, min S = 130, min V = 132; max H = 106, max S = 179, max V = 169 # Blue
        self.hmn = 95
        self.smn = 130
        self.vmn = 132
        self.hmx = 106
        self.smx = 179
        self.vmx = 169

    def start(self):
        x_coefficient = 1.0
        y_coefficient = 1.0
        serial_command = SerialCommand()
        while True:
            ret, frame = self.cap.read()
            frame = cv2.flip(frame, 1)
            height, width, channels = frame.shape
            x_reference, y_reference = width / 2, height

            # # converting to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hue, sat, val = cv2.split(hsv)

            hthresh = cv2.inRange(numpy.array(hue), numpy.array(self.hmn), numpy.array(self.hmx))
            sthresh = cv2.inRange(numpy.array(sat), numpy.array(self.smn), numpy.array(self.smx))
            vthresh = cv2.inRange(numpy.array(val), numpy.array(self.vmn), numpy.array(self.vmx))

            # AND h s and v
            tracking = cv2.bitwise_and(hthresh, cv2.bitwise_and(sthresh, vthresh))

            # Some morpholigical filtering
            dilation = cv2.dilate(tracking, self.kernel, iterations=1)
            closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, self.kernel)
            closing = cv2.GaussianBlur(closing, (5, 5), 0)

            # Detect circles using HoughCircles
            circles = cv2.HoughCircles(closing, cv2.HOUGH_GRADIENT, 2, 120, param1=120, param2=50, minRadius=10,
                                       maxRadius=0)
            # circles = np.uint16(np.around(circles))

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
                            cv2.line(frame, (int(x_reference), int(y_reference)), (x, y), (255, 0, 0), 2)
                            cv2.putText(frame, "{}".format(int(degree)), (x, int(y - 20)),
                                        cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                        1.5, (0, 0, 255))

                            cv2.putText(frame, "{}".format(int(distance)), (x, int(y + 60)),
                                        cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                        1.5, (0, 0, 255))
                            if r < 30:
                                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                                cv2.circle(frame, (x, y), 1, (0, 255, 0), 5)
                            elif r > 35:
                                cv2.circle(frame, (x, y), r, (0, 255, 255), 2)
                                cv2.circle(frame, (x, y), 1, (0, 255, 255), 5)
                        else:
                            # Remove print and send x, y, r to MicroController ;)
                            serial_command.send("X: {}, Y: {}, R: {}, Degree: {}, Distance: {}".format(x, y, r, degree, distance))
                            print("X: {}, Y: {}, R: {}, Degree: {}, Distance: {}".format(x, y, r, degree, distance))

            # Show the result in frames
            if self._test:
                cv2.imshow('HueComp', hthresh)
                cv2.imshow('SatComp', sthresh)
                cv2.imshow('ValComp', vthresh)
                cv2.imshow('closing', closing)
            cv2.imshow('tracking', frame)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q")]:
                break
            if key in [ord("r"), ord("R")]:
                self.reset()
            if key in [ord("c"), ord("C")]:
                c = Calibre(frame)
                c.start()
                if c.get_value():
                    self.hmn, self.smn, self.vmn, self.hmx, self.smx, self.vmx = c.get_value()

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()
