import cv2
import mediapipe as mp
import serial
import time

# Setup UART 
try:
    ser = serial.Serial('COM3', 115200, timeout=1)
    time.sleep(2)
except Exception as e:
    print("Error opening serial port: ", e)
    exit()

# Setup MediaPipe
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

last_state = None

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        frame = cv2.flip(frame, 1)
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            current_state = "LED_ON"
        else:
            current_state = "LED_OFF"

        if current_state != last_state:
            if current_state == "LED_ON":
                # Frame : Header (0xFF), Command (0x01), Terminator (0xFE) => Turn on LED
                frame_to_send = bytes([0xFF, 0x01, 0xFE])
                print(f"LED ON : {frame_to_send}")
            else:
                # Frame : Header (0xFF), Command (0x00), Terminator (0xFE) => Shutdown LED
                frame_to_send = bytes([0xFF, 0x00, 0xFE])
                print(f"LED OFF : {frame_to_send}")
            try:
                ser.write(frame_to_send)
                print(f"Sent frame: {[hex(b) for b in frame_to_send]}")
            except Exception as e:
                print("Error writing to serial port:", e)
            last_state = current_state

        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    ser.close()