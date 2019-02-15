import cv2
import numpy


class Calibre(object):
    def __init__(self, frame) -> None:
        super().__init__()

        cv2.namedWindow("Calibre")
        cv2.setMouseCallback("Calibre", self.click_and_crop)
        self.frame = frame
        self.clone = self.frame.copy()
        self.x_start = 0
        self.y_start = 0
        self.x_end = 0
        self.y_end = 0
        self.cropping = False
        self.getROI = False
        self.hsvRoi = None
        self.finish = False

    def start(self) -> None:
        while True:
            frame = self.frame.copy()
            if not self.cropping and not self.getROI:
                cv2.imshow("Calibre", frame)

            elif self.cropping and not self.getROI:
                cv2.rectangle(frame, (self.x_start, self.y_start), (self.x_end, self.y_end), (0, 255, 0), 2)
                cv2.imshow("Calibre", frame)

            elif not self.cropping and self.getROI:
                cv2.rectangle(frame, (self.x_start, self.y_start), (self.x_end, self.y_end), (0, 255, 0), 2)
                cv2.imshow("Calibre", frame)

            key = cv2.waitKey(1) & 0xFF

            # if the 'r' key is pressed, reset the cropping region
            if key in [ord("r"), ord("R")]:
                self.frame = self.clone.copy()
                self.getROI = False
            elif key in [ord("p"), ord("P")]:
                self.preview()
            # if the 'c' key is pressed, break from the loop
            elif key in [ord("c"), ord("C")]:
                break

        cv2.destroyWindow("Calibre")

    def click_and_crop(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x_start, self.y_start, self.x_end, self.y_end = x, y, x, y
            self.cropping = True

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.cropping:
                self.x_end, self.y_end = x, y

        elif event == cv2.EVENT_LBUTTONUP:
            self.x_end, self.y_end = x, y
            self.cropping = False
            self.getROI = True

    def preview(self):
        refPt = [(self.x_start, self.y_start), (self.x_end, self.y_end)]
        if len(refPt) == 2:
            roi = self.clone[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
            cv2.imshow("ROI", roi)

            self.hsvRoi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            print('min H = {}, min S = {}, min V = {}; max H = {}, max S = {}, max V = {}'.format(
                self.hsvRoi[:, :, 0].min(),
                self.hsvRoi[:, :, 1].min(),
                self.hsvRoi[:, :, 2].min(),
                self.hsvRoi[:, :, 0].max(),
                self.hsvRoi[:, :, 1].max(),
                self.hsvRoi[:, :, 2].max()))

            lower = numpy.array([self.hsvRoi[:, :, 0].min(), self.hsvRoi[:, :, 1].min(), self.hsvRoi[:, :, 2].min()])
            upper = numpy.array([self.hsvRoi[:, :, 0].max(), self.hsvRoi[:, :, 1].max(), self.hsvRoi[:, :, 2].max()])

            image_to_thresh = self.clone
            hsv = cv2.cvtColor(image_to_thresh, cv2.COLOR_BGR2HSV)

            kernel = numpy.ones((3, 3), numpy.uint8)
            # for red color we need to masks.
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            cv2.imshow("Mask", mask)
            cv2.waitKey(0)
            cv2.destroyWindow("Mask")
            cv2.destroyWindow("ROI")
            self.finish = True

    def get_value(self):
        if self.finish:
            return (self.hsvRoi[:, :, 0].min(),
                    self.hsvRoi[:, :, 1].min(),
                    self.hsvRoi[:, :, 2].min(),
                    self.hsvRoi[:, :, 0].max(),
                    self.hsvRoi[:, :, 1].max(),
                    self.hsvRoi[:, :, 2].max())
        return None
