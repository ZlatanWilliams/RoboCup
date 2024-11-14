# Import necessary libraries
from controller import Robot, Keyboard, Motion
import math
import cv2
import numpy as np
from ultralytics import YOLO
import time

# Define PID Controller class for smooth head movement
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
        self.turnLeft10 = Motion('../../motions/TurnLeft10.motion')
        self.turnLeft10v2 = Motion('../../motions/TurnLeft10_V2.motion')
        self.turnLeft20 = Motion('../../motions/TurnLeft20.motion')
        self.turnLeft30 = Motion('../../motions/TurnLeft30.motion')
        self.turnLeft40 = Motion('../../motions/TurnLeft40.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnLeft180 = Motion('../../motions/TurnLeft180.motion')
        self.turnRight10 = Motion('../../motions/TurnRight10.motion')
        self.turnRight10v2 = Motion('../../motions/TurnRight10_V2.motion')
        self.turnRight40 = Motion('../../motions/TurnRight40.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')
        self.Shoot = Motion('../../motions/Shoot.motion')
        self.RightShoot = Motion('../../motions/RightShoot.motion')
        self.wipeForhead = Motion('../../motions/WipeForehead.motion')
        self.LongPass = Motion('../../motions/LongPass.motion')
        self.StandUpFromBack = Motion('../../motions/StandUpFromBack.motion')
        self.StandUpFromFront = Motion('../../motions/StandUpFromFront.motion')
        self.OpenArms = Motion('../../motions/OpenArms.motion')

    def startMotion(self, motion):
        # Interrupt current motion
        if self.currentlyPlaying is not None and not self.currentlyPlaying.isOver():
            self.currentlyPlaying.stop()
        # Start new motion
        motion.play()
        self.currentlyPlaying = motion

    # The accelerometer axes are oriented as on the real robot
    # However, the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    # The gyro axes are oriented as on the real robot
    # However, the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        # z value is meaningless due to the orientation of the Gyro
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))

    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    # The InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printFootSensors(self):
        fsv = []  # force sensor values

        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())

        left = []
        right = []

        newtonsLeft = 0
        newtonsRight = 0

        # The coefficients were calibrated against the real
        # robot so as to obtain realistic sensor values.
        left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
        left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
        left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
        left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

        right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
        right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
        right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
        right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left

        for i in range(0, len(left)):
            left[i] = max(min(left[i], 25), 0)
            right[i] = max(min(right[i], 25), 0)
            newtonsLeft += left[i]
            newtonsRight += right[i]

        print('----------foot sensors----------')
        print('+ left ---- right +')
        print('+-------+ +-------+')
        print('|' + str(round(left[0], 1)) +
              '  ' + str(round(left[1], 1)) +
              '| |' + str(round(right[0], 1)) +
              '  ' + str(round(right[1], 1)) +
              '|  front')
        print('| ----- | | ----- |')
        print('|' + str(round(left[3], 1)) +
              '  ' + str(round(left[2], 1)) +
              '| |' + str(round(right[3], 1)) +
              '  ' + str(round(right[2], 1)) +
              '|  back')
        print('+-------+ +-------+')
        print('total: %f Newtons, %f kilograms'
              % ((newtonsLeft + newtonsRight), ((newtonsLeft + newtonsRight) / 9.81)))

    def printFootBumpers(self):
        ll = self.lfootlbumper.getValue()
        lr = self.lfootrbumper.getValue()
        rl = self.rfootlbumper.getValue()
        rr = self.rfootrbumper.getValue()
        print('----------foot bumpers----------')
        print('+ left ------ right +')
        print('+--------+ +--------+')
        print('|' + str(ll) + '  ' + str(lr) + '| |' + str(rl) + '  ' + str(rr) + '|')
        print('|        | |        |')
        print('|        | |        |')
        print('+--------+ +--------+')

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()

        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        for y in range(0, height // scaled):
            line = ''
            for x in range(0, width // scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255  # rescale between 0 and 9
                line = line + str(int(gray))
            print(line)

    def setAllLedsColor(self, rgb):
        # these leds take RGB values
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # ear leds are single color (blue)
        # and take values between 0 - 255
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            # Get motor min and max position
            minPosition = self.minPhalanxMotorPosition[i]
            maxPosition = self.maxPhalanxMotorPosition[i]

            # Ensure min position is non-negative
            if minPosition < 0:
                minPosition = 0.0

            # Clamp angle within min and max positions
            clampedAngle = max(min(angle, maxPosition), minPosition)

            if self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def printHelp(self):
        print('----------nao_demo_python----------')
        print('Use the keyboard to control the robot')
        print('(The 3D window needs to be focused)')
        print('[Up][Down]: move one step forward/backwards')
        print('[<-][->]: side step left/right')
        print('[Shift] + [<-][->]: turn left/right')
        print('[U]: print ultrasound sensors')
        print('[A]: print accelerometers')
        print('[G]: print gyros')
        print('[S]: print gps')
        print('[I]: print inertial unit (roll/pitch/yaw)')
        print('[F]: print foot sensors')
        print('[B]: print foot bumpers')
        print('[Home][End]: print scaled top/bottom camera image')
        print('[PageUp][PageDown]: open/close hands')
        print('[7][8][9]: change all leds RGB color')
        print('[0]: turn all leds off')
        print('[T]: perform Tai chi movements')
        print('[W]: wipe its forehead')
        print('[H]: print this help message')
        print('[L]: toggle ball tracking')

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)
        # Current active camera
        self.active_camera = 'top'

        # Get camera dimensions
        self.width = self.cameraTop.getWidth()
        self.height = self.cameraTop.getHeight()
        print(f"Camera dimensions: {self.width}x{self.height}")

        # accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controllable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # get phalanx motor tags
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            l_motor = self.getDevice("LPhalanx%d" % (i + 1))
            r_motor = self.getDevice("RPhalanx%d" % (i + 1))
            self.lphalanx.append(l_motor)
            self.rphalanx.append(r_motor)

            if r_motor is not None:
                maxPos = r_motor.getMaxPosition()
                minPos = r_motor.getMinPosition()
                # Ensure min position is non-negative
                if minPos < 0:
                    minPos = 0.0
                self.maxPhalanxMotorPosition.append(maxPos)
                self.minPhalanxMotorPosition.append(minPos)
            else:
                self.maxPhalanxMotorPosition.append(0.0)
                self.minPhalanxMotorPosition.append(0.0)

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # Head motor setup
        self.head_yaw = self.getDevice('HeadYaw')
        self.head_pitch = self.getDevice('HeadPitch')
        self.position_yaw_sensor = self.head_yaw.getPositionSensor()
        self.position_yaw_sensor.enable(self.timeStep)
        self.position_pitch_sensor = self.head_pitch.getPositionSensor()
        self.position_pitch_sensor.enable(self.timeStep)
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
        self.ball_class_id = 32  # Class ID for sports ball in COCO dataset

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = None
        self.ballTrackingEnabled = False  # Variable to control ball tracking
        # Initialize devices and motions
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.printHelp()

    # Method to get image from current active camera
    def get_camera_image(self):
        camera = self.cameraTop if self.active_camera == 'top' else self.cameraBottom
        image_data = camera.getImage()
        image = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        # Resize for processing
        target_width = 160
        target_height = 120
        image = cv2.resize(image, (target_width, target_height))
        return image

    # Method to switch between top and bottom cameras based on ball position
    def switch_camera(self):
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

    # Method to update head position to track the ball
    def update_head_position(self):
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

    # Method to process ball detections and update tracking state
    def process_ball_detection(self, detections, image):
        annotated_image = image.copy()
        self.ball_detected = False

        if len(detections) == 0:
            print("No ball detected")
            return annotated_image

        for detection in detections:
            boxes = detection.boxes.cpu().numpy()
            for box in boxes:
                class_id = int(box.cls[0])
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

    # Main loop
    def run(self):
        self.StandInit.setLoop(True)
        self.StandInit.play()
        self.currentlyPlaying = self.StandInit

        # until a key is pressed
        key = -1
        while self.step(self.timeStep) != -1:
            key = self.keyboard.getKey()
            if key > 0:
                break

        self.handWave.setLoop(False)

        while True:
            key = self.keyboard.getKey()

            if key == Keyboard.LEFT:
                self.startMotion(self.sideStepLeft)
            elif key == Keyboard.RIGHT:
                self.startMotion(self.sideStepRight)
            elif key == Keyboard.UP:
                self.startMotion(self.forwards50)
            elif key == Keyboard.DOWN:
                self.startMotion(self.backwards)
            elif key == (Keyboard.LEFT | Keyboard.SHIFT):
                self.startMotion(self.turnLeft10v2)
            elif key == (Keyboard.RIGHT | Keyboard.SHIFT):
                self.startMotion(self.turnRight10v2)
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
            elif key == ord('6'):
                self.startMotion(self.turnRight60)
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
                self.ballTrackingEnabled = not self.ballTrackingEnabled
                if self.ballTrackingEnabled:
                    print('Ball tracking enabled')
                else:
                    print('Ball tracking disabled')
            elif key == ord('Z'):
                # Show top camera view (for debugging)
                image = self.get_camera_image()
                cv2.imshow("Top Camera View", image)
                cv2.waitKey(1)
            elif key == ord('X'):
                # Switch to bottom camera and show view
                self.active_camera = 'bottom'
                image = self.get_camera_image()
                cv2.imshow("Bottom Camera View", image)
                cv2.waitKey(1)

            # Ball tracking logic
            if self.ballTrackingEnabled:
                self.frame_counter += 1
                if self.frame_counter % self.process_every_n_frames != 0:
                    if self.step(self.timeStep) == -1:
                        break
                    continue

                # Get camera image
                image = self.get_camera_image()

                # Run detection
                results = self.model.predict(image, classes=[self.ball_class_id], max_det=1)

                # Process detections
                annotated_image = self.process_ball_detection(results, image)

                # Update head tracking
                self.update_head_position()

                # Check if we need to switch cameras
                self.switch_camera()

                # Display results (optional)
                cv2.imshow("Ball Tracking", annotated_image)
                cv2.waitKey(1)

            if self.step(self.timeStep) == -1:
                break

# Create the Robot instance and run main loop
robot = Nao()
robot.run()
