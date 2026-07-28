"""
BUOC 1: Test nhanh phat hien nguoi bang HOG (khong can tai model)
Chay truc tiep tren Raspberry Pi qua VNC:
    python3 1_quick_test_hog.py

Yeu cau:
    pip3 install opencv-python
"""

import cv2
import time

# Khoi tao HOG people detector co san trong OpenCV
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

# Mo webcam (0 = camera mac dinh, doi thanh 1,2... neu co nhieu camera)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Khong mo duoc webcam. Kiem tra ket noi hoac index camera (0/1/2).")
    exit()

prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Giam kich thuoc de tang toc do xu ly tren Pi
    small = cv2.resize(frame, (400, 300))

    # Phat hien nguoi (tra ve danh sach bounding box)
    boxes, weights = hog.detectMultiScale(
        small, winStride=(8, 8), padding=(8, 8), scale=1.05
    )

    # Ve bounding box va tinh tam khung hinh nguoi
    h, w = small.shape[:2]
    frame_center_x = w // 2

    for (x, y, bw, bh) in boxes:
        cv2.rectangle(small, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cx = x + bw // 2
        cy = y + bh // 2
        cv2.circle(small, (cx, cy), 4, (0, 0, 255), -1)

        # Offset so voi tam khung hinh -> dung de dieu khien servo pan/tilt sau nay
        offset_x = cx - frame_center_x
        print(f"Nguoi tai ({cx},{cy}) | offset_x so voi tam: {offset_x}")

    # Tinh FPS
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(small, f"FPS: {fps:.1f}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow("HOG Person Detection", small)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
