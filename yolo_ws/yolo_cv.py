import cv2
from ultralytics import YOLO

model = YOLO("best.pt")
cap = cv2.VideoCapture("video.mp4")

while True:
    ret, frame = cap.read()
    if not ret:
        print("비디오 종료 또는 읽기 실패")
        break

    results = model(frame)[0]

    # 박스 그리기 반복문
    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        cls = int(box.cls[0])
        conf = box.conf[0]

        label = f"{results.names[cls]} {conf:.2f}"
        
        # 사각형과 텍스트 그리기
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
        cv2.putText(frame, label, (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    cv2.imshow("YOLO Detection", frame)

    # 키 입력 대기 (q를 누르면 종료)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()