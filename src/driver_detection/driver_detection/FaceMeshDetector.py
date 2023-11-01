import cv2
import mediapipe as mp
import time
import numpy as np
import itertools

# Constants

# https://github.com/google/mediapipe/blob/7c28c5d58ffbcb72043cbe8c9cc32b40aaebac41/mediapipe/python/solutions/face_mesh_connections.py

# Face indices 
FACE_OVAL=[ 10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136, 172, 58, 132, 93, 234, 127, 162, 21, 54, 103,67, 109]

# Lips indices
LIPS=[ 61, 146, 91, 181, 84, 17, 314, 405, 321, 375,291, 308, 324, 318, 402, 317, 14, 87, 178, 88, 95,185, 40, 39, 37,0 ,267 ,269 ,270 ,409, 415, 310, 311, 312, 13, 82, 81, 42, 183, 78 ]
LOWER_LIPS =[61, 146, 91, 181, 84, 17, 314, 405, 321, 375, 291, 308, 324, 318, 402, 317, 14, 87, 178, 88, 95]
UPPER_LIPS=[ 185, 40, 39, 37,0 ,267 ,269 ,270 ,409, 415, 310, 311, 312, 13, 82, 81, 42, 183, 78]
# Left eyes indices 
LEFT_EYE =[ 362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398 ]
LEFT_EYEBROW =[ 336, 296, 334, 293, 300, 276, 283, 282, 295, 285 ]
LEFT_IRIS = [474, 475, 476, 477]

# Right eyes indices
RIGHT_EYE=[ 33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161 , 246 ]  
RIGHT_EYEBROW=[ 70, 63, 105, 66, 107, 55, 65, 52, 53, 46 ]
RIGHT_IRIS = [469, 470, 471, 472]


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
            mesh_coords = self.normalizeToPixelCoordinates(res_img, self.faceMeshLM)
            res_img =self.fillPolyTrans(res_img, [mesh_coords[p] for p in RIGHT_EYE], (0, 0, 255), opacity=0.4)
            res_img =self.fillPolyTrans(res_img, [mesh_coords[p] for p in LEFT_EYE], (0, 0, 255), opacity=0.4)
            for faceLms in self.faceMeshLM.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(res_img, faceLms, self.mpFaceMesh.FACEMESH_CONTOURS, self.drawSpec, self.drawSpec)
        return res_img

    def fillPolyTrans(self, img, points, color, opacity):
        overlay = img.copy()
        list_to_np_array = np.array(points, dtype=np.int32)
        cv2.fillPoly(overlay,[list_to_np_array], color )
        result_img = cv2.addWeighted(overlay, opacity, img, 1 - opacity, 0)
        cv2.polylines(result_img, [list_to_np_array], True, color,1, cv2.LINE_AA)
        return result_img

    def normalizeIrisToPixelCoordinates(self, iris_point, image_shape):
        iris_center_x = 0.0        
        iris_center_y = 0.0
        for landmark in iris_point:
            iris_center_x = iris_center_x + landmark.x
            iris_center_y = iris_center_y + landmark.y
        iris_center_x = iris_center_x/4 * image_shape[0]
        iris_center_y = iris_center_y/4 * image_shape[1]
        return [iris_center_x, iris_center_y]

    def normalizeToPixelCoordinates(self, img, results):
        image_height, image_width= img.shape[:2]
        mesh_coord = [(point.x * image_width, point.y * image_height) for point in results.multi_face_landmarks[0].landmark]
        return mesh_coord
    
    def normalizeEyesToPixelCoordinates(self, img, results):
        image_height, image_width= img.shape[:2]
        mesh_coord = [[point.x * image_width, point.y * image_height] for point in results.multi_face_landmarks[0].landmark]
        return mesh_coord

    def findIris(self, img, draw=True):
        res_img = img.copy()
        imgRGB = cv2.cvtColor(res_img, cv2.COLOR_BGR2RGB)
        self.irisLM = self.faceMesh.process(imgRGB)
        if self.irisLM.multi_face_landmarks:
            for faceLms in self.irisLM.multi_face_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(res_img, faceLms, self.mpFaceMesh.FACEMESH_IRISES, None, self.mp_drawing_styles.get_default_face_mesh_iris_connections_style())
        return res_img
    
    def getIrisPosition(self, img):
        image_height, image_width = img.shape[:2]
        if self.irisLM.multi_face_landmarks:
            for faceLms in self.irisLM.multi_face_landmarks:
                right_iris = faceLms.landmark[RIGHT_IRIS[0]:RIGHT_IRIS[len(RIGHT_IRIS)-1]]
                right_iris_center_x, right_iris_center_y = self.normalizeIrisToPixelCoordinates(right_iris, [image_height, image_width])
                # print(f"Right eye:\n x: {int(right_iris_center_x)}, y: {int(right_iris_center_y)}")
                
                left_iris = faceLms.landmark[LEFT_IRIS[0]:LEFT_IRIS[len(LEFT_IRIS)-1]]
                left_iris_center_x, left_iris_center_y = self.normalizeIrisToPixelCoordinates(left_iris, [image_height, image_width])
                # print(f"Left eye:\n x: {int(left_iris_center_x)}, y: {int(left_iris_center_y)}")
        
            return [right_iris_center_x, right_iris_center_y, left_iris_center_x, left_iris_center_y]
        return [0.0, 0.0, 0.0, 0.0]
    
    def getFaceKeypointPositions(self, img):
        res_img = img.copy()
        result = []
        if self.faceMeshLM.multi_face_landmarks:
            mesh_coords = self.normalizeToPixelCoordinates(res_img, self.faceMeshLM)
            nose = mesh_coords[1]
            chin = mesh_coords[199]
            left_eye_corner = mesh_coords[33]
            right_eye_corner = mesh_coords[263]
            left_mouth_corner = mesh_coords[61]
            right_mouth_corner = mesh_coords[291]
            result = [nose[0], nose[1], chin[0], chin[1], left_eye_corner[0], left_eye_corner[1], right_eye_corner[0], right_eye_corner[1],
                      left_mouth_corner[0], left_mouth_corner[1], right_mouth_corner[0], right_mouth_corner[1]]
        return result
    
    def getEyePosition(self, img):
        res_img = img.copy()
        result = []
        if self.faceMeshLM.multi_face_landmarks:
            mesh_coords = self.normalizeEyesToPixelCoordinates(res_img, self.faceMeshLM)
            left_eye_coords = [mesh_coords[p] for p in LEFT_EYE]
            right_eye_coords = [mesh_coords[p] for p in RIGHT_EYE]
            right_eye_top = right_eye_coords[12]
            right_eye_bottom = right_eye_coords[4]
            right_eye_left = right_eye_coords[8]
            right_eye_right = right_eye_coords[0]
            left_eye_top = left_eye_coords[12]
            left_eye_bottom = left_eye_coords[4]
            left_eye_left = left_eye_coords[8]
            left_eye_right = left_eye_coords[0]
            result = [right_eye_top[0], right_eye_top[1], right_eye_bottom[0], right_eye_bottom[1],
                       right_eye_left[0], right_eye_left[1], right_eye_right[0], right_eye_right[1],
                      left_eye_top[0], left_eye_top[1], left_eye_bottom[0], left_eye_bottom[1],
                       left_eye_left[0], left_eye_left[1], left_eye_right[0], left_eye_right[1]]
        return result

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