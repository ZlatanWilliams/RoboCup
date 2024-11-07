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
        # Current active camera
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
        # Enable position control for head motors
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
        self.model = YOLO('yolov8n.pt')
        
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
        if self.display:
            self.display.attachCamera(self.camera_top)
    
    def get_camera_image(self):
        """Get image from current active camera"""
        camera = self.camera_top if self.active_camera == 'top' else self.camera_bottom
        image_data = camera.getImage()
        image = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        
        # Resize for processing
        target_width = 160
        target_height = 120
        image = cv2.resize(image, (target_width, target_height))
        
        return image
    
    def switch_camera(self):
        """Switch between top and bottom cameras based on ball position"""
        if not self.ball_detected:
            return
            
        # Get normalized y position (0 at top, 1 at bottom)
        _, y = self.last_ball_position
        norm_y = y / self.height
        
        # Switch camera based on ball position
        if self.active_camera == 'top' and norm_y > 0.8:
            self.active_camera = 'bottom'
            print("Switching to bottom camera")
        elif self.active_camera == 'bottom' and norm_y < 0.2:
            self.active_camera = 'top'
            print("Switching to top camera")
    
    def update_head_position(self):
        """Update head position to track the ball"""
        if not self.ball_detected:
            self.lost_ball_counter += 1
            if self.lost_ball_counter > self.max_lost_frames:
                # Return head to center position if ball is lost
                self.head_yaw.setPosition(0.0)
                self.head_pitch.setPosition(0.0)
            return
        
        self.lost_ball_counter = 0
        
        # Get normalized ball position (-1 to 1)
        x, y = self.last_ball_position
        norm_x = (x / self.width * 2) - 1
        norm_y = (y / self.height * 2) - 1
        
        # Calculate desired head positions using PID
        current_yaw = self.position_yaw_sensor.getValue()
        current_pitch = self.position_pitch_sensor.getValue()
        
        # Update yaw (horizontal)
        yaw_error = -norm_x  # Negative because positive yaw is left
        new_yaw = current_yaw + self.yaw_pid.update(yaw_error)
        new_yaw = np.clip(new_yaw, self.min_yaw, self.max_yaw)
        
        # Update pitch (vertical)
        pitch_error = norm_y
        new_pitch = current_pitch + self.pitch_pid.update(pitch_error)
        new_pitch = np.clip(new_pitch, self.min_pitch, self.max_pitch)
        
        # Set new positions
        self.head_yaw.setPosition(new_yaw)
        self.head_pitch.setPosition(new_pitch)
        
        print(f"Head position - Yaw: {new_yaw:.2f}, Pitch: {new_pitch:.2f}")
    
    def process_ball_detection(self, detections, image):
        """Process ball detections and update tracking state"""
        annotated_image = image.copy()
        self.ball_detected = False
        
        # print("Step 1: ", detections)
        if len(detections) == 0:
            print("\nNo ball detected")
            return annotated_image
        
        for detection in detections:
            boxes = detection.boxes.cpu().numpy()
            for box in boxes:
                class_id = int(box.cls[0])
                print('class id: ', class_id)
                if class_id == self.ball_class_id:
                    confidence = box.conf[0]
                    if confidence < 0.5:
                        continue
                    
                    x1, y1, x2, y2 = box.xyxy[0].astype(int)
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    self.last_ball_position = (center_x, center_y)
                    self.ball_detected = True
                    
                    # Draw detection visualization
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated_image, (center_x, center_y), 2, (0, 0, 255), -1)
                    label = f'Ball {confidence:.2%}'
                    cv2.putText(annotated_image, label, (x1, y1 - 5),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # Draw active camera indicator
                    camera_text = f"Camera: {self.active_camera.upper()}"
                    cv2.putText(annotated_image, camera_text, (5, 15),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
        return annotated_image
    
    def run(self):
        """Main control loop"""
        print("Starting ball tracking...")
        
        while self.robot.step(self.timestep) != -1:
            self.frame_counter += 1
            
            if self.frame_counter % self.process_every_n_frames != 0:
                continue
            
            # Get camera image
            image = self.get_camera_image()
            
            # Run detection
            results = self.model.predict(image, classes=[self.ball_class_id], max_det=1)
            
            # Process detections
            print('Detecting ball')
            annotated_image = self.process_ball_detection(results, image)
            
            # Update head tracking
            print('Updating Head')
            self.update_head_position()
            
            # Check if we need to switch cameras
            self.switch_camera()
            
            # Display results if available
            if self.display:
                display_image = cv2.resize(annotated_image, (self.width, self.height))
                display_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2BGRA)
                self.display.setImage(display_image.tobytes())

class PIDController:
    """Simple PID controller for smooth head movement"""
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

if __name__ == "__main__":
    main()