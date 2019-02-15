import cv2
import numpy
from Calibre import Calibre

# True only for test code ;)
TEST = False


class BallDetection(object):
    def __init__(self):
        super(BallDetection, self).__init__()
        self.cap = cv2.VideoCapture(0)
        self.kernel = numpy.ones((5, 5), numpy.uint8)

        self.hmn = 0
        self.smn = 197
        self.vmn = 214
        self.hmx = 179
        self.smx = 218
        self.vmx = 255

    def reset(self):
        self.hmn = 0
        self.smn = 197
        self.vmn = 214
        self.hmx = 179
        self.smx = 218
        self.vmx = 255

    def start(self):
        while True:
            ret, frame = self.cap.read()

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
                    if TEST:
                        if r < 30:
                            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                            cv2.circle(frame, (x, y), 1, (0, 255, 0), 5)
                        elif r > 35:
                            cv2.circle(frame, (x, y), r, (0, 255, 255), 2)
                            cv2.circle(frame, (x, y), 1, (0, 255, 255), 5)
                    else:
                        if r:
                            # Remove print and send x, y, r to raspberry ;)
                            print("X: {}, Y: {}, R: {}".format(x, y, r))

            # you can use the 'buzz' variable as a trigger to switch some GPIO lines on Rpi :)
            # print buzz
            # if buzz:
            # put your GPIO line here

            # Show the result in frames
            if TEST:
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
