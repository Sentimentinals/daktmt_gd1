"""
BUOC 2: Phat hien nguoi bang MobileNet-SSD (nhanh + chinh xac hon HOG)

TRUOC KHI CHAY, tai 2 file model ve cung thu muc voi script nay
(chay tren Pi, can Pi co internet):

    wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt -O MobileNetSSD_deploy.prototxt
    wget https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel -O MobileNetSSD_deploy.caffemodel

Neu link tren loi (Github doi ho so), cu tim tu khoa:
    "MobileNetSSD_deploy.prototxt" va "MobileNetSSD_deploy.caffemodel" tren Github/Google

Chay:
    python3 2_person_detect_ssd.py
"""

import cv2
import time

PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"
CONF_THRESHOLD = 0.5

# 21 class ma MobileNet-SSD nay duoc train (chi lay class "person")
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle",
           "bus", "car", "cat", "chair", "cow", "diningtable", "dog",
           "horse", "motorbike", "person", "pottedplant", "sheep", "sofa",
           "train", "tvmonitor"]
PERSON_IDX = CLASSES.index("person")

net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Khong mo duoc webcam.")
    exit()

prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    frame_center_x = w // 2
    best_conf = 0
    best_box = None

    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        class_id = int(detections[0, 0, i, 1])

        if confidence > CONF_THRESHOLD and class_id == PERSON_IDX:
            box = detections[0, 0, i, 3:7] * [w, h, w, h]
            (x1, y1, x2, y2) = box.astype("int")

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"person {confidence:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            # Chon nguoi co do tin cay cao nhat lam muc tieu de robot "nhin theo"
            if confidence > best_conf:
                best_conf = confidence
                best_box = (cx, cy)

    if best_box:
        cx, cy = best_box
        offset_x = cx - frame_center_x
        offset_y = cy - (h // 2)
        print(f"Muc tieu chinh: ({cx},{cy}) | offset_x={offset_x} offset_y={offset_y}")
        # --> Day chinh la 2 gia tri se dua vao vong dieu khien PID
        #     cho servo pan (truc X, xoay co) va tilt (truc Y, ngua/cui dau)

    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow("MobileNet-SSD Person Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
