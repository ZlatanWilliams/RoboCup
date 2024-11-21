from controller import Robot, Supervisor
import cv2
import numpy as np
import time
from sklearn.cluster import DBSCAN

class RobotDetection:
    def __init__(self):
        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Camera setup
        self.camera_top = self.robot.getDevice('CameraTop')
        self.camera_bottom = self.robot.getDevice('CameraBottom')
        self.camera_top.enable(self.timestep)
        self.camera_bottom.enable(self.timestep)
        self.width = self.camera_top.getWidth()
        self.height = self.camera_top.getHeight()
        
        # Adjusted Color detection ranges for better detection
        self.red_lower = np.array([0, 100, 50])  # Expanded the range for red
        self.red_upper = np.array([10, 255, 255])
        self.blue_lower = np.array([90, 100, 50])  # Expanded the range for blue
        self.blue_upper = np.array([140, 255, 255])
        
        # DBSCAN clustering parameters
        self.epsilon = 30  # Distance threshold for clustering
        self.min_samples = 1  # Minimum number of points to form a cluster
        self.min_contour_area = 50  # Lowered minimum contour area for smaller objects

    def cluster_contours(self, centers):
        """Cluster contour centers using DBSCAN, only if centers is not empty"""
        if len(centers) == 0:
            return np.array([])  # Return an empty array if no centers are detected
        clustering = DBSCAN(eps=self.epsilon, min_samples=self.min_samples).fit(centers)
        return clustering.labels_

    def detect_and_annotate(self, image, camera_name):
        """Detect teammates and opponents, return detection results, and log processing time"""
        start_preprocess = time.time()
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = {"Teammate": [], "Opponent": []}
        end_preprocess = time.time()
        
        # Detect teammates (red)
        start_detection = time.time()
        red_mask = cv2.inRange(hsv_image, self.red_lower, self.red_upper)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        red_centers = [
            (int(x + w / 2), int(y + h / 2)) 
            for c in red_contours 
            for x, y, w, h in [cv2.boundingRect(c)]
            if cv2.contourArea(c) >= self.min_contour_area
        ]
        red_labels = self.cluster_contours(red_centers)
        for label in set(red_labels):
            members = [red_centers[i] for i in range(len(red_centers)) if red_labels[i] == label]
            if members:
                detections["Teammate"].append(np.mean(members, axis=0).astype(int))

        # Detect opponents (blue)
        blue_mask = cv2.inRange(hsv_image, self.blue_lower, self.blue_upper)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        blue_centers = [
            (int(x + w / 2), int(y + h / 2)) 
            for c in blue_contours 
            for x, y, w, h in [cv2.boundingRect(c)]
            if cv2.contourArea(c) >= self.min_contour_area
        ]
        blue_labels = self.cluster_contours(blue_centers)
        for label in set(blue_labels):
            members = [blue_centers[i] for i in range(len(blue_centers)) if blue_labels[i] == label]
            if members:
                detections["Opponent"].append(np.mean(members, axis=0).astype(int))
        
        end_detection = time.time()

        # Log detection status
        if not detections["Teammate"] and not detections["Opponent"]:
            print(f"{camera_name}: No detections")
        else:
            for label, members in detections.items():
                for pos in members:
                    print(f"{camera_name}: {label} detected at position {pos}")
        
        # Log processing times
        preprocess_time = (end_preprocess - start_preprocess) * 1000
        detection_time = (end_detection - start_detection) * 1000
        print(f"{camera_name} - Preprocess: {preprocess_time:.1f} ms, Detection: {detection_time:.1f} ms")

        return detections

    def process_robot_detection(self, detections, image):
        """Annotate image with detections"""
        annotated_image = image.copy()
        
        for label, members in detections.items():
            for (x_center, y_center) in members:
                # Draw detection marker and label
                color = (0, 0, 255) if label == "Teammate" else (255, 0, 0)
                cv2.circle(annotated_image, (x_center, y_center), 10, color, -1)
                cv2.putText(annotated_image, label, (x_center - 20, y_center - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return annotated_image
    
    def run(self):
        """Main control loop"""
        while self.robot.step(self.timestep) != -1:
            # Get top camera image
            top_image_data = self.camera_top.getImage()
            if top_image_data:
                top_image = np.frombuffer(top_image_data, np.uint8).reshape((self.height, self.width, 4))
                top_image = cv2.cvtColor(top_image, cv2.COLOR_BGRA2BGR)
                
                print("\nProcessing top camera...")
                top_detections = self.detect_and_annotate(top_image, "Top Camera")
                top_annotated_image = self.process_robot_detection(top_detections, top_image)
                cv2.imshow("Top Camera Detection", top_annotated_image)
            
            # Get bottom camera image
            bottom_image_data = self.camera_bottom.getImage()
            if bottom_image_data:
                bottom_image = np.frombuffer(bottom_image_data, np.uint8).reshape((self.height, self.width, 4))
                bottom_image = cv2.cvtColor(bottom_image, cv2.COLOR_BGRA2BGR)
                
                print("\nProcessing bottom camera...")
                bottom_detections = self.detect_and_annotate(bottom_image, "Bottom Camera")
                bottom_annotated_image = self.process_robot_detection(bottom_detections, bottom_image)
                cv2.imshow("Bottom Camera Detection", bottom_annotated_image)
            
            cv2.waitKey(1)

if __name__ == "__main__":
    robot_detection = RobotDetection()
    robot_detection.run()
