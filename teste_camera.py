import cv2
from picamera2 import Picamera2
camera = Picamera2()
camera.configure(camera.create_preview_configuration(
    main={"format": "RGB888", "size": (320, 240)}
))
camera.start()

while True:
    frame = camera.capture_array()
    cv2.imshow("Frame", frame)
