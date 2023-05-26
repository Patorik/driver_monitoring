import cv2
import mediapipe as mp
import time
import itertools

class FaceMeshDetector:
    def __init__(self, static_image_mode=False, max_num_faces=1, refine_landmarks=True, min_detection_confidence=0.5, min_tracking_confidence=0.5):
        self.static_image_mode = static_image_mode
        self.max_num_faces = max_num_faces
        self.refine_landmarks = refine_landmarks
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence

        self.mpFaceMesh = mp.solutions.face_mesh
        self.faceMesh = self.mpFaceMesh.FaceMesh(self.static_image_mode, self.max_num_faces, self.refine_landmarks, self.min_detection_confidence, self.min_tracking_confidence)
        self.mpDraw = mp.solutions.drawing_utils
        self.drawSpec = self.mpDraw.DrawingSpec(thickness=1, circle_radius=2)
        self.mp_drawing_styles = mp.solutions.drawing_styles

    def findIris(self, img, draw=True):
        res_img = img.copy()
        imgRGB = cv2.cvtColor(res_img, cv2.COLOR_BGR2RGB)
        self.irisLM = self.faceMesh.process(imgRGB)
        # print(results)
        # LEFT_EYE_INDEXES = list(set(itertools.chain(self.mpFaceMesh.FACEMESH_LEFT_EYE)))
        # RIGHT_EYE_INDEXES = list(set(itertools.chain(self.mpFaceMesh.FACEMESH_RIGHT_EYE)))
        # print(f"Left eye indexes:{LEFT_EYE_INDEXES}")
        # print(f"Right eye indexes:{RIGHT_EYE_INDEXES}")

        if self.irisLM.multi_face_landmarks:
            for faceLms in self.irisLM.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(res_img, faceLms, self.mpFaceMesh.FACEMESH_IRISES, None, self.mp_drawing_styles.get_default_face_mesh_iris_connections_style())
        return res_img
    
    def getIrisPosition(self, img, draw=True):
        irisLM_pos = []
        if self.irisLM.multi_face_landmarks:
            for faceLms in self.irisLM.multi_face_landmarks:
                for id, lm in enumerate(faceLms.landmark):
                    if id in [469, 470, 471, 472]:                              # https://github.com/google/mediapipe/blob/7c28c5d58ffbcb72043cbe8c9cc32b40aaebac41/mediapipe/python/solutions/face_mesh_connections.py
                        # print(lm)
                        print("RIGHT EYE IRIS")
                        ih, iw, ic = img.shape
                        x, y = int(lm.x * iw), int(lm.y * ih)
                        print(f"ID:{id}, {x}, {y}")
                    if id in [474, 475, 476, 477]:
                        print("LEFT EYE IRIS")
                        ih, iw, ic = img.shape
                        x, y = int(lm.x * iw), int(lm.y * ih)
                        print(f"ID:{id}, {x}, {y}")

                    



def main():
    cap = cv2.VideoCapture(0)
    cTime = pTime = 0
    detector = FaceMeshDetector()
    while True:
        success, frame = cap.read()

        frame = detector.findIris(frame)
        detector.getIrisPosition(frame)
                
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