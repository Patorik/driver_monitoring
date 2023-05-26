import cv2
import mediapipe as mp
import time

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
        results = self.faceMesh.process(imgRGB)
        # print(results)

        if results.multi_face_landmarks:
            for faceLms in results.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(res_img, faceLms, self.mpFaceMesh.FACEMESH_IRISES, self.drawSpec, self.mp_drawing_styles.get_default_face_mesh_iris_connections_style())
                # for id, lm in enumerate(faceLms.landmark):
                #     #print(lm)
                #     ih, iw, ic = img.shape
                #     x, y = int(lm.x * iw), int(lm.y * ih)
                #     print(id, x, y)
        return res_img


def main():
    cap = cv2.VideoCapture(0)
    cTime = pTime = 0
    detector = FaceMeshDetector()
    while True:
        success, frame = cap.read()

        detector.findIris(frame)
                
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