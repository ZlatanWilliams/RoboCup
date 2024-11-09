from controller import Robot, Keyboard, Motion
import math
import cv2
import numpy as np
from ultralytics import YOLO
import time

# Define the PID controller class
class PIDController:
    """Simple PID controller for smoothing head movements"""
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

# Define the main Nao robot class
class Nao(Robot):
    PHALANX_MAX = 8

    # Load motion files
    def loadMotionFiles(self):
        self.StandInit = Motion('../../motions/StandInit.motion')
        self.handWave = Motion('../../motions/HandWave.motion')
        self.forwards = Motion('../../motions/Forwards.motion')
        self.forwards50 = Motion('../../motions/Forwards50.motion')
        self.forwardsSprint = Motion('../../motions/ForwardsSprint.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.SidePass_Left = Motion('../../motions/SidePass_Left.motion')
        self.SidePass_Right = Motion('../../motions/SidePass_Right.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')
        self.Shoot = Motion('../../motions/Shoot.motion')
        self.RightShoot = Motion('../../motions/RightShoot.motion')
        self.wipeForhead = Motion('../../motions/WipeForehead.motion')
        self.LongPass = Motion('../../motions/LongPass.motion')
        self.StandUpFromBack = Motion('../../motions/StandUpFromBack.motion')
        self.StandUpFromFront = Motion('../../motions/StandUpFromFront.motion')
        self.OpenArms = Motion('../../motions/OpenArms.motion')

    def startMotion(self, motion):
        # Interrupt current action
        if self.currentlyPlaying is not None and not self.currentlyPlaying.isOver():
            self.currentlyPlaying.stop()
        # Start new action
        motion.play()
        self.currentlyPlaying = motion

    def printHelp(self):
        print('----------nao_demo----------')
        print('Use the keyboard to control the robot (one action at a time)')
        print('(Window must be focused on the 3D view)')
        print('[Up][Down]: Take a step forward/backward')
        print('[<-][->]: Side step left/right')
        print('[Shift] + [<-][->]: Turn left/right')
        print('[U]: Print ultrasonic sensor information')
        print('[A]: Print accelerometer information')
        print('[G]: Print gyroscope information')
        print('[S]: Print GPS information')
        print('[I]: Print inertial unit information (roll/pitch/yaw)')
        print('[F]: Print foot sensor information')
        print('[B]: Print foot bumper information')
        print('[Home][End]: Print zoomed top/bottom camera images')
        print('[PageUp][PageDown]: Open/close hands')
        print('[7][8][9]: Change RGB color of all LEDs')
        print('[0]: Turn off all LEDs')
        print('[T]: Execute SidePass_Right action')
        print('[W]: Wipe forehead')
        print('[K]: Execute Shoot action')
        print('[P]: Execute LongPass action')
        print('[N]: Stand up from back')
        print('[M]: Stand up from front')
        print('[1]: Execute RightShoot action')
        print('[2]: Execute ForwardsSprint action')
        print('[3]: Execute Forwards action')
        print('[4]: Execute HandWave action')
        print('[5]: Execute OpenArms action')
        print('[H]: Print this help message')
        print('[L]: Toggle ball tracking')

    def findAndEnableDevices(self):
        # Get the time step of the current world
        self.timeStep = int(self.getBasicTimeStep())

        # Head motors
        self.head_yaw = self.getDevice('HeadYaw')
        self.head_pitch = self.getDevice('HeadPitch')

        # Enable position sensors for head motors
        self.head_yaw_sensor = self.head_yaw.getPositionSensor()
        self.head_yaw_sensor.enable(self.timeStep)

        self.head_pitch_sensor = self.head_pitch.getPositionSensor()
        self.head_pitch_sensor.enable(self.timeStep)

        # Cameras
        self.cameraTop = self.getDevice('CameraTop')
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom = self.getDevice('CameraBottom')
        self.cameraBottom.enable(4 * self.timeStep)

        self.width = self.cameraTop.getWidth()
        self.height = self.cameraTop.getHeight()
        print(f"Camera dimensions: {self.width} x {self.height}")

        # Other sensors and devices
        # Accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # Gyroscope
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.timeStep)

        # GPS
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)

        # Inertial unit
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # Ultrasonic sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # Foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # Foot bumper sensors
        self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # LEDs
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # Finger motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # Assuming the same motor position range for both hands
            if self.rphalanx[i] is not None:
                self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
                self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())
            else:
                self.maxPhalanxMotorPosition.append(0.0)
                self.minPhalanxMotorPosition.append(0.0)

        # Shoulder motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # Keyboard control
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)


    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = None
        self.tracking = False  # Tracking function is initially off
        self.rotating = False  # Rotation state

        # Initialize devices and variables
        self.findAndEnableDevices()
        self.loadMotionFiles()
        # self.printHelp()

        # Initialize PID controllers for head movement
        self.pid_yaw = PIDController(kp=0.002, ki=0.0, kd=0.001)
        self.pid_pitch = PIDController(kp=0.002, ki=0.0, kd=0.001)

        # Load the YOLO model
        # Replace with absolute path if needed
        self.yolo_model = YOLO('yolov8n.pt')  # Ensure model availability

        # Print model class names for debugging
        self.class_names = self.yolo_model.model.names
        print("Model class names:", self.class_names)

        # Retrieve class ID for soccer ball
        self.desired_class_id = None
        for class_id, class_name in self.class_names.items():
            if class_name.lower() in ['sports ball', 'ball', 'soccer ball']:
                self.desired_class_id = class_id
                break
        if self.desired_class_id is None:
            print("Error: Soccer ball class not found in the model.")
        else:
            print(f"Soccer ball class ID: {self.desired_class_id}")

    def run(self):
        # Start the StandInit action
        self.StandInit.setLoop(True)
        self.StandInit.play()
        self.currentlyPlaying = self.StandInit

        # Set minimum confidence threshold
        MIN_CONFIDENCE = 0.15  # Adjust as needed

        while self.step(self.timeStep) != -1:
            # Process keyboard input
            key = self.keyboard.getKey()
            while key > 0:
                if key == Keyboard.LEFT:
                    self.startMotion(self.sideStepLeft)
                elif key == Keyboard.RIGHT:
                    self.startMotion(self.sideStepRight)
                elif key == Keyboard.UP:
                    self.startMotion(self.forwards50)
                elif key == Keyboard.DOWN:
                    self.startMotion(self.backwards)
                elif key == (Keyboard.LEFT | Keyboard.SHIFT):
                    self.startMotion(self.turnLeft60)
                elif key == (Keyboard.RIGHT | Keyboard.SHIFT):
                    self.startMotion(self.turnRight60)
                elif key == ord('A'):
                    self.printAcceleration()
                elif key == ord('G'):
                    self.printGyro()
                elif key == ord('S'):
                    self.printGps()
                elif key == ord('I'):
                    self.printInertialUnit()
                elif key == ord('F'):
                    self.printFootSensors()
                elif key == ord('B'):
                    self.printFootBumpers()
                elif key == ord('U'):
                    self.printUltrasoundSensors()
                elif key == ord('T'):
                    self.startMotion(self.SidePass_Right)
                elif key == ord('W'):
                    self.startMotion(self.wipeForhead)
                elif key == ord('K'):
                    self.startMotion(self.Shoot)
                elif key == ord('P'):
                    self.startMotion(self.LongPass)
                elif key == ord('N'):
                    self.startMotion(self.StandUpFromBack)
                elif key == ord('M'):
                    self.startMotion(self.StandUpFromFront)
                elif key == Keyboard.HOME:
                    self.printCameraImage(self.cameraTop)
                elif key == Keyboard.END:
                    self.printCameraImage(self.cameraBottom)
                elif key == Keyboard.PAGEUP:
                    self.setHandsAngle(0.96)
                elif key == Keyboard.PAGEDOWN:
                    self.setHandsAngle(0.0)
                elif key == ord('1'):
                    self.startMotion(self.RightShoot)
                elif key == ord('2'):
                    self.startMotion(self.forwardsSprint)
                elif key == ord('3'):
                    self.startMotion(self.forwards)
                elif key == ord('4'):
                    self.startMotion(self.handWave)
                elif key == ord('5'):
                    self.startMotion(self.OpenArms)
                elif key == ord('7'):
                    self.setAllLedsColor(0xff0000)  # red
                elif key == ord('8'):
                    self.setAllLedsColor(0x00ff00)  # green
                elif key == ord('9'):
                    self.setAllLedsColor(0x0000ff)  # blue
                elif key == ord('0'):
                    self.setAllLedsColor(0x000000)  # off
                elif key == ord('H'):
                    self.printHelp()
                elif key == ord('L'):
                    self.tracking = not self.tracking
                    if self.tracking:
                        print("Ball tracking enabled.")
                    else:
                        print("Ball tracking disabled.")
                key = self.keyboard.getKey()

            if self.tracking:
                # Capture top camera image
                image_top = self.cameraTop.getImage()
                image_top = np.frombuffer(image_top, np.uint8).reshape((self.height, self.width, 4))
                image_top = cv2.cvtColor(image_top, cv2.COLOR_BGRA2BGR)

                # Capture bottom camera image
                image_bottom = self.cameraBottom.getImage()
                image_bottom = np.frombuffer(image_bottom, np.uint8).reshape((self.height, self.width, 4))
                image_bottom = cv2.cvtColor(image_bottom, cv2.COLOR_BGRA2BGR)

                # Optional: Display camera images for debugging
                # cv2.imshow("Top Camera", image_top)
                # cv2.imshow("Bottom Camera", image_bottom)
                # cv2.waitKey(1)

                # Run YOLO detection
                results_top = self.yolo_model(image_top, conf=MIN_CONFIDENCE)
                results_bottom = self.yolo_model(image_bottom, conf=MIN_CONFIDENCE)

                # Initialize variables
                ball_detected = False
                ball_x = 0
                ball_y = 0
                camera_used = 'top'

                # Check top camera
                for result in results_top:
                    for box in result.boxes:
                        class_id = int(box.cls[0])
                        class_name = self.class_names[class_id]
                        confidence = box.conf[0]
                        print(f"[Top Camera] Detected {class_name} with confidence {confidence:.2f}")
                        if class_id == self.desired_class_id and confidence >= MIN_CONFIDENCE:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            ball_x = (x1 + x2) / 2
                            ball_y = (y1 + y2) / 2
                            ball_detected = True
                            camera_used = 'top'
                            print("Ball detected by top camera.")
                            break
                    if ball_detected:
                        break

                # If not detected by top, check bottom camera
                if not ball_detected:
                    for result in results_bottom:
                        for box in result.boxes:
                            class_id = int(box.cls[0])
                            class_name = self.class_names[class_id]
                            confidence = box.conf[0]
                            print(f"[Bottom Camera] Detected {class_name} with confidence {confidence:.2f}")
                            if class_id == self.desired_class_id and confidence >= MIN_CONFIDENCE:
                                x1, y1, x2, y2 = map(int, box.xyxy[0])
                                ball_x = (x1 + x2) / 2
                                ball_y = (y1 + y2) / 2
                                ball_detected = True
                                camera_used = 'bottom'
                                print("Ball detected by bottom camera.")
                                break
                        if ball_detected:
                            break

                if ball_detected:
                    # Stop rotation
                    if self.rotating:
                        self.rotating = False
                        self.StandInit.play()
                        self.currentlyPlaying = self.StandInit
                        print("Stopping rotation as ball is detected.")

                    # Calculate deviation of the ball from image center
                    error_x = (self.width / 2) - ball_x
                    error_y = (self.height / 2) - ball_y

                    # Update PID controller
                    yaw_adjust = self.pid_yaw.update(error_x)
                    pitch_adjust = self.pid_pitch.update(error_y)

                    # Get current head position
                    current_yaw = self.head_yaw_sensor.getValue()
                    current_pitch = self.head_pitch_sensor.getValue()

                    # Calculate new position
                    new_yaw = current_yaw + yaw_adjust
                    new_pitch = current_pitch - pitch_adjust  # Subtract for image coordinates

                    # Adjust head based on vertical position of the ball
                    norm_y = ball_y / self.height
                    if camera_used == 'bottom':
                        if norm_y > 0.5:
                            new_pitch -= 0.05  # Look downward
                            print("Adjusting head pitch downward.")
                        elif norm_y < 0.2:
                            new_pitch += 0.05  # Look upward
                            print("Adjusting head pitch upward.")

                    # Limit within motor range
                    new_yaw = max(min(new_yaw, self.head_yaw.getMaxPosition()), self.head_yaw.getMinPosition())
                    new_pitch = max(min(new_pitch, self.head_pitch.getMaxPosition()), self.head_pitch.getMinPosition())

                    # Set new head position
                    self.head_yaw.setPosition(new_yaw)
                    self.head_pitch.setPosition(new_pitch)

                    print(f"Ball detected by {camera_used} camera at ({ball_x:.2f}, {ball_y:.2f}). Adjusting head to ({new_yaw:.2f}, {new_pitch:.2f}).")

                    # Switch camera based on ball position
                    if camera_used == 'top' and norm_y > 0.8:
                        self.active_camera = 'bottom'
                        print("Switching to bottom camera")
                    elif camera_used == 'bottom' and norm_y < 0.2:
                        self.active_camera = 'top'
                        print("Switching to top camera")

                    # Move the robot towards the ball
                    if not self.currentlyPlaying == self.forwards:
                        self.startMotion(self.forwards)
                        print("Moving forwards towards the ball.")
                else:
                    print("Ball not detected by any camera.")
                    # Use ultrasonic sensors to assist with positioning
                    # Get distance data from ultrasonic sensors
                    if len(self.us) >= 2:
                        left_dist = self.us[0].getValue()
                        right_dist = self.us[1].getValue()
                    else:
                        left_dist, right_dist = (None, None)

                    print('-----ultrasonic sensors-----')
                    print(f'left: {left_dist:.2f} m, right: {right_dist:.2f} m')

                    # Define distance threshold
                    FRONT_THRESHOLD = 1.0  # 1 meter

                    if left_dist is not None and right_dist is not None:
                        if left_dist < FRONT_THRESHOLD and right_dist < FRONT_THRESHOLD:
                            # Ball is in front, move forwards
                            if not self.rotating or (self.currentlyPlaying != self.forwards):
                                self.startMotion(self.forwards)
                                self.rotating = False
                                print("Moving forwards towards the ball based on ultrasonic sensors.")
                        elif left_dist < FRONT_THRESHOLD:
                            # Ball is on the left, turn right
                            self.startMotion(self.turnRight60)
                            self.rotating = False
                            print("Turning right towards the ball based on ultrasonic sensors.")
                        elif right_dist < FRONT_THRESHOLD:
                            # Ball is on the right, turn left
                            self.startMotion(self.turnLeft60)
                            self.rotating = False
                            print("Turning left towards the ball based on ultrasonic sensors.")
                        else:
                            # Ball is behind, turn 180 degrees
                            # Since there is no 180-degree action, perform two 60-degree turns to rotate
                            self.startMotion(self.turnLeft60)
                            self.rotating = False
                            print("Turning left to search for the ball behind based on ultrasonic sensors.")
                    elif left_dist is not None:
                        if left_dist < FRONT_THRESHOLD:
                            self.startMotion(self.turnRight60)
                            self.rotating = False
                            print("Turning right towards the ball based on left ultrasonic sensor.")
                        else:
                            # No detection, continue rotating
                            pass
                    elif right_dist is not None:
                        if right_dist < FRONT_THRESHOLD:
                            self.startMotion(self.turnLeft60)
                            self.rotating = False
                            print("Turning left towards the ball based on right ultrasonic sensor.")
                        else:
                            # No detection, continue rotating
                            pass
                    else:
                        # No sensor data, continue rotating
                        pass

                    # If the robot is not rotating, start rotating
                    if not self.rotating:
                        print("Starting rotation to find the ball.")
                        self.rotating = True
                        self.turnLeft60.play()
                        self.currentlyPlaying = self.turnLeft60
                    else:
                        # Check if the rotation action is completed
                        if self.currentlyPlaying.isOver():
                            print("Rotation motion over, restarting rotation.")
                            self.turnLeft60.play()
                            self.currentlyPlaying = self.turnLeft60

            # Optional: Add a small delay to prevent excessive CPU usage
            time.sleep(0.01)
        else:
            pass

    def useUltrasonicSensors(self):
        # Retrieve distance information from the ultrasonic sensors
        distances = [sensor.getValue() for sensor in self.us]
        left_dist, right_dist = distances if len(distances) >= 2 else (None, None)

        print('-----ultrasonic sensors-----')
        print(f'left: {left_dist:.2f} m, right: {right_dist:.2f} m')

        # Basic navigation logic based on ultrasonic sensor data (example)
        if left_dist is not None and right_dist is not None:
            if left_dist < right_dist:
                # If the left side is closer, turn right
                self.startMotion(self.turnRight60)
                print("Turning right towards the ball based on ultrasonic sensors.")
            elif right_dist < left_dist:
                # If the right side is closer, turn left
                self.startMotion(self.turnLeft60)
                print("Turning left towards the ball based on ultrasonic sensors.")
            else:
                # If distances are similar, move forward
                self.startMotion(self.forwards)
                print("Moving forwards as ultrasonic distances are similar.")
        elif left_dist is not None:
            if left_dist < 0.5:
                self.startMotion(self.turnRight60)
                print("Turning right towards the ball based on left ultrasonic sensor.")
            else:
                self.startMotion(self.forwards)
                print("Moving forwards based on left ultrasonic sensor.")
        elif right_dist is not None:
            if right_dist < 0.5:
                self.startMotion(self.turnLeft60)
                print("Turning left towards the ball based on right ultrasonic sensor.")
            else:
                self.startMotion(self.forwards)
                print("Moving forwards based on right ultrasonic sensor.")


    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        # z value is meaningless due to the orientation of the Gyro
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))

    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printFootSensors(self):
        pass

    def printFootBumpers(self):
        pass

    def printUltrasoundSensors(self):
        dist = []
        for sensor in self.us:
            dist.append(sensor.getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right: %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # Define the scaling factor for the image
        width = camera.getWidth()
        height = camera.getHeight()

        # Read the RGB pixel values from the camera
        image = camera.getImage()

        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        for y in range(0, int(height / scaled)):
            line = ''
            for x in range(0, int(width / scaled)):
                # Get the gray level of the pixel at (x, y) and rescale it to a range from 0 to 9
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255
                line = line + str(int(gray))
            print(line)

    def setAllLedsColor(self, rgb):
        # Set the color of all LEDs using RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # The ear LEDs are monochromatic (blue)
        # Accepts a value between 0 and 255
        self.leds[5].set(rgb & 0xFF)  # Set the blue component for ear LEDs
        self.leds[6].set(rgb & 0xFF)  # Set the blue component for ear LEDs

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            # Get the minimum and maximum positions for the motor
            minPosition = self.minPhalanxMotorPosition[i]
            maxPosition = self.maxPhalanxMotorPosition[i]

            # Ensure the minimum position is non-negative
            if minPosition < 0:
                minPosition = 0.0

            # Clamp the angle to be between the minimum and maximum motor positions
            clampedAngle = max(min(angle, maxPosition), minPosition)

            # Set the position for each phalanx (fingers)
            if self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

robot = Nao()
robot.run()
