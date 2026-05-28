import cv2
import mediapipe as mp
import pyautogui
import time

SMOOTHING = 0.3
PINCH_THRESH = 0.05
DEBOUNCE_SEC = 0.4

SCREEN_W, SCREEN_H = pyautogui.size()
cx, cy = SCREEN_W // 2, SCREEN_H // 2
lastClick = 0

cap = cv2.VideoCapture(0)

with mp.solutions.hands.Hands(max_num_hands=1) as hands:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        rgb = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)
        if(res.multi_hand_landmarks):
            lm = res.multi_hand_landmarks[0].landmark
            cx = int(cx*(1-SMOOTHING) + lm[8].x*SCREEN_W*SMOOTHING)
            cy = int(cy*(1-SMOOTHING) + lm[8].y*SCREEN_W*SMOOTHING)
            pyautogui.moveTo(cx, cy, _pause=False)
            
            d = ((lm[4].x-lm[8].x)**2 + (lm[4].y)**2)**0.5
            if d < PINCH_THRESH and (time.time()-last_click) > DEBOUNCE_SEC:
                pyautogui.click()
                last_click = time.time()
        if cv2.waitKey(1) & 0xFF == ord('q'):
           break
cap.release()
cv2.destroyAllWindows()
