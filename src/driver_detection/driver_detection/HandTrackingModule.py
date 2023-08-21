import cv2
import mediapipe as mp
import time


class HandDetector():
    def __init__(self, mode=False, max_hands=2, model_complexity=1, detection_confidence=0.5, tracking_confidence=0.5):
        self.mode = mode
        self.max_hands = max_hands
        self.model_complexity = model_complexity
        self.detection_confidence = detection_confidence
        self.tracking_confidence = tracking_confidence

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.max_hands, self.model_complexity, self.detection_confidence, self.tracking_confidence)
        self.mpDraw = mp.solutions.drawing_utils


    def findHands(self, img, draw=True):
        res_img = img.copy()
        imgRGB = cv2.cvtColor(res_img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        #print(self.results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(res_img, handLms, self.mpHands.HAND_CONNECTIONS)
        return res_img

    def findPosition(self, img, handNo=0, draw=True):
        lmList = []

        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]

            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x*w), int(lm.y*h)
                #print(id, cx, cy)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx,cy), 10, (255, 0, 0), cv2.FILLED)
        
        return lmList
                
def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0)
    detector = HandDetector()
    while True:
        success, img = cap.read()

        detector.findHands(img)
        lmList = detector.findPosition(img)
        if len(lmList) > 0:
            print(lmList[14])

        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0,255,255), 3)

        cv2.imshow("Video", img)
        key = cv2.waitKey(1)
        if key==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
    