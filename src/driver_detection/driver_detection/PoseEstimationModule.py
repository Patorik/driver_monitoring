import cv2
import mediapipe as mp
import time

class PoseDetector:
    def __init__(self, mode=True, smooth_landmarks=False, enable_segmentation=False, smooth_segmentation=True, detection_confidence=0.5, tracking_confidence=0.5):
        self.mode = mode
        self.smooth_landmarks = smooth_landmarks
        self.enable_segmentation = enable_segmentation
        self.smooth_segmentation = smooth_segmentation
        self.detection_confidence = detection_confidence
        self.tracking_confidence = tracking_confidence

        self.mpPose = mp.solutions.pose
        self.pose = self.mpPose.Pose(self.mode, 1, self.smooth_landmarks, self.enable_segmentation, self.smooth_segmentation, self.detection_confidence, self.tracking_confidence)
        self.mpDraw = mp.solutions.drawing_utils

    def findPose(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.pose.process(imgRGB)
        #print(results.post_landmarks)

        if self.results.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(img, self.results.pose_landmarks, self.mpPose.POSE_CONNECTIONS)
        return img
            
    def getPosition(self, img, draw=True):
        lmList = []
        if self.results.pose_landmarks:
            for id, lm in enumerate(self.results.pose_landmarks.landmark):
                h, w, c = img.shape
                #print(id, lm)
                cx, cy = int(lm.x*w), int(lm.y*h)
                lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 7, (0, 0, 255), cv2.FILLED)
        return lmList


def main():
    cap = cv2.VideoCapture(0)
    cTime = pTime = 0
    detector = PoseDetector()
    while True:
        success, frame = cap.read()

        detector.findPose(frame)
        lmList = detector.getPosition(frame)
        # if len(lmList)>0:
        #     print(lmList)

        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime

        cv2.putText(frame, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 5, (0, 255, 255), 3)
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
        if key==ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()