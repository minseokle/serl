from indy7_env.camera.video_capture import VideoCapture
from indy7_env.camera.usb_cam_capture import USBCapture
import cv2

cap = VideoCapture(cap=USBCapture(name="usb", usb=0))

while True:
    frame = cap.read()
    cv2.imshow("frame", frame)
    cv2.waitKey(1)
