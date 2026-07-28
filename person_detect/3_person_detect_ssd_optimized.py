"""
BUOC 3: Ban toi uu cua MobileNet-SSD - tang FPS bang threading + frame skip

Yeu cau: cung thu muc phai co san
    - MobileNetSSD_deploy.prototxt
    - MobileNetSSD_deploy.caffemodel
(da tai o buoc truoc)

Chay:
    python3 3_person_detect_ssd_optimized.py
"""

import cv2
import time
import threading

PROTOTXT = "MobileNetSSD_deploy.prototxt"
MODEL = "MobileNetSSD_deploy.caffemodel"
CONF_THRESHOLD = 0.5
CAPTURE_W, CAPTURE_H = 320, 240   # giam resolution capture -> giam tai xu ly
DETECT_EVERY_N_FRAMES = 2         # chi chay detect moi 2 frame, frame con lai dung lai box cu

CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle",
           "bus", "car", "cat", "chair", "cow", "diningtable", "dog",
           "horse", "motorbike", "person", "pottedplant", "sheep", "sofa",
           "train", "tvmonitor"]
PERSON_IDX = CLASSES.index("person")


class ThreadedCamera:
    """Doc camera o thread rieng, luon giu frame moi nhat, khong bi block."""

    def __init__(self, src=0):
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # giam buffer -> giam do tre

        if not self.cap.isOpened():
            raise RuntimeError("Khong mo duoc webcam.")

        self.ret, self.frame = self.cap.read()
        self.stopped = False
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret, self.frame = ret, frame

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
        self.thread.join()
        self.cap.release()


net = cv2.dnn.readNetFromCaffe(PROTOTXT, MODEL)

camera = ThreadedCamera(0)  # doi so 0 neu can

frame_count = 0
last_boxes = []  # cache ket qua detect gan nhat de dung khi skip frame
prev_time = time.time()

try:
    while True:
        ret, frame = camera.read()
        if not ret or frame is None:
            continue

        h, w = frame.shape[:2]
        frame_count += 1

        # Chi chay inference (buoc nang nhat) moi N frame
        if frame_count % DETECT_EVERY_N_FRAMES == 0:
            blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
            net.setInput(blob)
            detections = net.forward()

            last_boxes = []
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                class_id = int(detections[0, 0, i, 1])
                if confidence > CONF_THRESHOLD and class_id == PERSON_IDX:
                    box = detections[0, 0, i, 3:7] * [w, h, w, h]
                    (x1, y1, x2, y2) = box.astype("int")
                    last_boxes.append((x1, y1, x2, y2, confidence))

        # Ve lai box (dung ket qua cache neu frame nay bi skip detect)
        frame_center_x = w // 2
        best_conf = 0
        best_box = None

        for (x1, y1, x2, y2, confidence) in last_boxes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"person {confidence:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            if confidence > best_conf:
                best_conf = confidence
                best_box = (cx, cy)

        if best_box:
            cx, cy = best_box
            offset_x = cx - frame_center_x
            offset_y = cy - (h // 2)
            print(f"Muc tieu: ({cx},{cy}) | offset_x={offset_x} offset_y={offset_y}")

        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        prev_time = curr_time
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Optimized SSD Person Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    camera.stop()
    cv2.destroyAllWindows()
