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

    def findFaceMesh(self, img, draw=True):
        res_img = img.copy()
        imgRGB = cv2.cvtColor(res_img, cv2.COLOR_BGR2RGB)
        self.faceMeshLM = self.faceMesh.process(imgRGB)

        if self.faceMeshLM.multi_face_landmarks:
            for faceLms in self.faceMeshLM.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(res_img, faceLms, self.mpFaceMesh.FACEMESH_CONTOURS, self.drawSpec, self.drawSpec)
        return res_img

    def findIris(self, img, draw=True):
        res_img = img.copy()
        imgRGB = cv2.cvtColor(res_img, cv2.COLOR_BGR2RGB)
        self.irisLM = self.faceMesh.process(imgRGB)

        if self.irisLM.multi_face_landmarks:
            for faceLms in self.irisLM.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(res_img, faceLms, self.mpFaceMesh.FACEMESH_IRISES, None, self.mp_drawing_styles.get_default_face_mesh_iris_connections_style())
        return res_img
    
    def getIrisPosition(self, img, draw=True):
        irisLM_pos = []
        ih, iw, ic = img.shape
        if self.irisLM.multi_face_landmarks:
            for faceLms in self.irisLM.multi_face_landmarks:
                right_iris = faceLms.landmark[474:478] # https://github.com/google/mediapipe/blob/7c28c5d58ffbcb72043cbe8c9cc32b40aaebac41/mediapipe/python/solutions/face_mesh_connections.py
                right_iris_center_x = 0
                right_iris_center_y = 0
                for rm in right_iris:
                    right_iris_center_x = right_iris_center_x + rm.x
                    right_iris_center_y = right_iris_center_y + rm.y
                right_iris_center_x = right_iris_center_x/4 * iw
                right_iris_center_y = right_iris_center_y/4 * ih
                # print(f"Right eye:\n x: {int(right_iris_center_x)}, y: {int(right_iris_center_y)}")
                
                left_iris = faceLms.landmark[469:473]
                left_iris_center_x = 0
                left_iris_center_y = 0
                for lm in left_iris:
                    left_iris_center_x = left_iris_center_x + lm.x
                    left_iris_center_y = left_iris_center_y + lm.y
                left_iris_center_x = left_iris_center_x/4 * iw
                left_iris_center_y = left_iris_center_y/4 * ih
                # print(f"Left eye:\n x: {int(left_iris_center_x)}, y: {int(left_iris_center_y)}")
        
            return [right_iris_center_x, right_iris_center_y, left_iris_center_x, left_iris_center_y]
        return [0.0, 0.0, 0.0, 0.0]

def main():
    cap = cv2.VideoCapture(0)
    cTime = pTime = 0
    detector = FaceMeshDetector()
    while True:
        success, frame = cap.read()

        frame = cv2.flip(frame, 1)

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