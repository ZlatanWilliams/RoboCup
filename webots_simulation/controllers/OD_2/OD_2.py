from controller import Robot
import cv2
import numpy as np
from ultralytics import YOLO
import time

class BallTrackingRobot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Camera setup
        self.camera_top = self.robot.getDevice('CameraTop')
        self.camera_bottom = self.robot.getDevice('CameraBottom')
        self.camera_top.enable(self.timestep)
        self.camera_bottom.enable(self.timestep)
        self.active_camera = 'top'
        
        # Get camera dimensions
        self.width = self.camera_top.getWidth()
        self.height = self.camera_top.getHeight()
        print(f"Camera dimensions: {self.width}x{self.height}")
        
        # Head motor setup
        self.head_yaw = self.robot.getDevice('HeadYaw')
        self.head_pitch = self.robot.getDevice('HeadPitch')
        self.position_yaw_sensor = self.head_yaw.getPositionSensor()
        self.position_yaw_sensor.enable(self.timestep)
        self.position_pitch_sensor = self.head_pitch.getPositionSensor()
        self.position_pitch_sensor.enable(self.timestep)
        self.head_yaw.setPosition(0.0)
        self.head_pitch.setPosition(0.0)
        
        # Get motor limits
        self.max_yaw = self.head_yaw.getMaxPosition()
        self.min_yaw = self.head_yaw.getMinPosition()
        self.max_pitch = self.head_pitch.getMaxPosition()
        self.min_pitch = self.head_pitch.getMinPosition()
        
        # PID control parameters
        self.yaw_pid = PIDController(kp=0.5, ki=0.1, kd=0.05)
        self.pitch_pid = PIDController(kp=0.5, ki=0.1, kd=0.05)
        
        # Load YOLO model
        self.model = YOLO('yolov8m.pt')
        
        # Processing parameters
        self.process_every_n_frames = 3
        self.frame_counter = 0
        
        # Ball tracking state
        self.last_ball_position = None
        self.ball_detected = False
        self.lost_ball_counter = 0
        self.max_lost_frames = 10
        self.ball_class_id = 32
        
        # Display setup
        self.display = self.robot.getDevice('display') if self.robot.getDevice('display') else None
        
    def get_camera_image(self):
        """Get images from both cameras"""
        # Get top camera image
        top_image_data = self.camera_top.getImage()
        top_image = np.frombuffer(top_image_data, np.uint8).reshape((self.height, self.width, 4))
        top_image = cv2.cvtColor(top_image, cv2.COLOR_BGRA2BGR)
        
        # Get bottom camera image
        bottom_image_data = self.camera_bottom.getImage()
        bottom_image = np.frombuffer(bottom_image_data, np.uint8).reshape((self.height, self.width, 4))
        bottom_image = cv2.cvtColor(bottom_image, cv2.COLOR_BGRA2BGR)
        
        # Debug: Save images to disk to verify they're different
        cv2.imwrite('top_camera.jpg', top_image)
        cv2.imwrite('bottom_camera.jpg', bottom_image)
        
        # Resize both images
        target_width = 160
        target_height = 120
        top_image = cv2.resize(top_image, (target_width, target_height))
        bottom_image = cv2.resize(bottom_image, (target_width, target_height))
        
        return top_image, bottom_image
    
    def detect_ball(self, image, camera_name):
        """Detect ball in image"""
        results = self.model.predict(image, classes=[self.ball_class_id], max_det=1)
        return self.process_ball_detection(results, image, camera_name)
    
    def process_ball_detection(self, detections, image, camera_name):
        """Process ball detections and update tracking state"""
        annotated_image = image.copy()
        ball_detected = False
        ball_position = None
        
        if len(detections) > 0:
            for detection in detections:
                boxes = detection.boxes.cpu().numpy()
                for box in boxes:
                    class_id = int(box.cls[0])
                    print(f'{camera_name} camera detected class: {class_id}')
                    
                    if class_id == self.ball_class_id:
                        confidence = box.conf[0]
                        print(f'{camera_name} camera ball confidence: {confidence}')
                        
                        if confidence < 0.3:
                            continue
                        
                        x1, y1, x2, y2 = box.xyxy[0].astype(int)
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        ball_detected = True
                        ball_position = (center_x, center_y)
                        
                        # Draw detection
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(annotated_image, (center_x, center_y), 2, (0, 0, 255), -1)
                        label = f'Ball {confidence:.2%}'
                        cv2.putText(annotated_image, label, (x1, y1 - 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw camera indicator
        cv2.putText(annotated_image, f"Camera: {camera_name}", (5, 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Save annotated images for debugging
        cv2.imwrite(f'{camera_name}_annotated.jpg', annotated_image)
        
        return annotated_image, ball_detected, ball_position
    
    def update_head_position(self, ball_position):
        """Update head position to track the ball"""
        if ball_position is None:
            self.lost_ball_counter += 1
            if self.lost_ball_counter > self.max_lost_frames:
                self.head_yaw.setPosition(0.0)
                self.head_pitch.setPosition(0.0)
            return
        
        self.lost_ball_counter = 0
        x, y = ball_position
        
        # Get normalized ball position (-1 to 1)
        norm_x = (x / self.width * 2) - 1
        norm_y = (y / self.height * 2) - 1
        
        # Calculate desired head positions using PID
        current_yaw = self.position_yaw_sensor.getValue()
        current_pitch = self.position_pitch_sensor.getValue()
        
        # Update yaw (horizontal)
        yaw_error = -norm_x
        new_yaw = current_yaw + self.yaw_pid.update(yaw_error)
        new_yaw = np.clip(new_yaw, self.min_yaw, self.max_yaw)
        
        # Update pitch (vertical)
        pitch_error = norm_y
        new_pitch = current_pitch + self.pitch_pid.update(pitch_error)
        new_pitch = np.clip(new_pitch, self.min_pitch, self.max_pitch)
        
        # Set new positions
        self.head_yaw.setPosition(new_yaw)
        self.head_pitch.setPosition(new_pitch)
    
    def run(self):
        """Main control loop"""
        print("Starting ball tracking...")
        
        while self.robot.step(self.timestep) != -1:
            self.frame_counter += 1
            
            if self.frame_counter % self.process_every_n_frames != 0:
                continue
            
            # Get images from both cameras
            top_image, bottom_image = self.get_camera_image()
            
            # Process both cameras
            print("\nProcessing top camera...")
            top_annotated, top_detected, top_position = self.detect_ball(top_image, "top")
            
            print("\nProcessing bottom camera...")
            bottom_annotated, bottom_detected, bottom_position = self.detect_ball(bottom_image, "bottom")
            
            # Use detections from either camera
            if top_detected:
                print("Ball detected in top camera")
                self.ball_detected = True
                self.last_ball_position = top_position
                self.active_camera = 'top'
                display_image = top_annotated
            elif bottom_detected:
                print("Ball detected in bottom camera")
                self.ball_detected = True
                self.last_ball_position = bottom_position
                self.active_camera = 'bottom'
                display_image = bottom_annotated
            else:
                print("No ball detected in either camera")
                self.ball_detected = False
                self.last_ball_position = None
                display_image = top_annotated  # Default to top camera view
            
            # Update head position
            self.update_head_position(self.last_ball_position)
            
            # Display results
            if self.display:
                display_image = cv2.resize(display_image, (self.width, self.height))
                display_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2BGRA)
                self.display.setImage(display_image.tobytes())

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error):
        self.integral += error
        derivative = error - self.prev_error
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        self.prev_error = error
        return output

def main():
    try:
        robot = BallTrackingRobot()
        robot.run()
    except KeyboardInterrupt:
        print("\nStopping robot gracefully...")
    except Exception as e:
        print(f"Error occurred: {e}")
        raise e

if __name__ == "__main__":
    main()