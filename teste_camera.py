import cv2
from picamera2 import Picamera2
camera = Picamera2()
camera.configure(camera.create_preview_configuration(
    main={"format": "RGB888", "size": (1280, 720)}
))
camera.start()

while True:
    frame = camera.capture_array()
    cv2.imshow("Frame", frame)
    # falta isto:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.stop()
cv2.destroyAllWindows()