from ultralytics import YOLO
import cv2 as cv
import matplotlib.pyplot as plt

model = YOLO('yolov8m.pt')
cap = cv.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
        
    results = model.predict(frame)
    #qresults.show()
    image = results[0].plot()

    image_bgr = cv.cvtColor(image, cv.COLOR_RGB2BGR)
    cv.imshow('results', image)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()