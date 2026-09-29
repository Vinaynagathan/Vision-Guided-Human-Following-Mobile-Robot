"""
Vision subsystem for the human-following robot.

Based on the project paper:
- YOLOv8n
- 1280x720 camera input
- confidence threshold = 0.5
- select the largest detected person
- calculate bounding-box centroid
- left / center / right region mapping
- USB serial commands at 9600 bps
- commands: L, F, R, S
- command transmission approximately every 40-50 ms

NOTE:
The paper describes the architecture and parameters but does not provide
the original Python source. This is a reconstructed reference
implementation, not the original project code.
"""

import argparse
import time

import cv2
import serial
from ultralytics import YOLO


def choose_command(cx: float, frame_width: int) -> str:
    """
    Map the person's centroid into three horizontal regions.

    Left  : x < W/3
    Center: W/3 <= x <= 2W/3
    Right : x > 2W/3
    """
    left_boundary = frame_width / 3.0
    right_boundary = 2.0 * frame_width / 3.0

    if cx < left_boundary:
        return "L"
    elif cx > right_boundary:
        return "R"
    return "F"


def largest_person(result):
    """Return the largest detected person box or None."""
    if result.boxes is None or len(result.boxes) == 0:
        return None

    candidates = []

    for box in result.boxes:
        class_id = int(box.cls[0].item())

        # COCO class 0 = person
        if class_id != 0:
            continue

        x1, y1, x2, y2 = box.xyxy[0].tolist()
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        confidence = float(box.conf[0].item())

        candidates.append((area, confidence, x1, y1, x2, y2))

    if not candidates:
        return None

    return max(candidates, key=lambda item: item[0])


def draw_detection(frame, detection, command):
    if detection is None:
        cv2.putText(
            frame,
            "No person detected",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
        )
        return

    _, confidence, x1, y1, x2, y2 = detection

    x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))

    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)

    cv2.putText(
        frame,
        f"Person {confidence:.2f}",
        (x1, max(25, y1 - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
    )

    cv2.putText(
        frame,
        f"Command: {command}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (255, 255, 0),
        2,
    )


def draw_regions(frame):
    height, width = frame.shape[:2]

    left = int(width / 3)
    right = int(2 * width / 3)

    cv2.line(frame, (left, 0), (left, height), (255, 255, 0), 1)
    cv2.line(frame, (right, 0), (right, height), (255, 255, 0), 1)

    cv2.putText(
        frame, "LEFT", (10, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
    )
    cv2.putText(
        frame, "CENTER", (left + 15, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
    )
    cv2.putText(
        frame, "RIGHT", (right + 15, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--port", type=str, required=True)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--model", type=str, default="yolov8n.pt")
    args = parser.parse_args()

    model = YOLO(args.model)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    # Paper target: 1280x720 input.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        cap.release()
        raise RuntimeError(f"Could not open serial port {args.port}: {exc}") from exc

    last_send = 0.0
    send_interval = 0.045  # ~45 ms, within the paper's 40-50 ms range

    last_command = "S"

    print("Vision subsystem started.")
    print(f"Camera index : {args.camera}")
    print(f"Serial port  : {args.port}")
    print(f"Baud rate    : {args.baud}")
    print("Press Q to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame could not be read.")
                break

            # Fixed input size from the paper.
            resized = cv2.resize(frame, (1280, 720))

            results = model.predict(
                source=resized,
                imgsz=1280,
                conf=0.5,
                verbose=False,
            )

            detection = largest_person(results[0])

            command = "S"

            if detection is not None:
                _, _, x1, y1, x2, y2 = detection
                cx = (x1 + x2) / 2.0
                command = choose_command(cx, resized.shape[1])

            now = time.perf_counter()

            # Approx. 40-50 ms command interval.
            if now - last_send >= send_interval:
                if command != last_command:
                    ser.write(command.encode("ascii"))
                    ser.flush()
                    last_command = command
                elif command != "S":
                    # Periodic refresh keeps Arduino command timeout alive.
                    ser.write(command.encode("ascii"))
                    ser.flush()

                if command == "S" and last_command != "S":
                    ser.write(b"S")
                    ser.flush()
                    last_command = "S"

                last_send = now

            draw_regions(resized)
            draw_detection(resized, detection, command)

            cv2.imshow("Human-Following Robot - YOLOv8n", resized)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break

    finally:
        # Leave robot in a safe state before exiting.
        try:
            ser.write(b"S")
            ser.flush()
        except serial.SerialException:
            pass

        ser.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
