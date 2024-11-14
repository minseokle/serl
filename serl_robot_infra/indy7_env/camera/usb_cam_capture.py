import cv2


class USBCapture:
    def __init__(self, name, usb, dim=(640, 480), fps=15, depth=False):
        assert depth == False
        self.name = name
        self.depth = depth
        self.cap = cv2.VideoCapture(usb)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, dim[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, dim[1])
        self.cap.set(cv2.CAP_PROP_FPS, fps)


    def read(self):
        ret, frame = self.cap.read()
        if ret:
            return True, frame
        else:
            return False, None

    def close(self):
        self.cap.release()
