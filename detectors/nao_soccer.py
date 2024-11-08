from controller import Robot, Keyboard, Motion
import torch
from PIL import Image
import numpy as np
import cv2
from ultralytics import YOLO

class Nao (Robot):
    PHALANX_MAX = 8 
        
    # load motion files
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
        self.Shake = Motion('../../motions/ShakeHead.motion')
        self.Run = Motion('../../motions/Run.motion')

    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    # the gyro axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        # z value is meaningless due to the orientation of the Gyro
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))

    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
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
            clampedAngle = angle
            if clampedAngle > self.maxPhalanxMotorPosition[i]:
                clampedAngle = self.maxPhalanxMotorPosition[i]
            elif clampedAngle < self.minPhalanxMotorPosition[i]:
                clampedAngle = self.minPhalanxMotorPosition[i]

            if len(self.rphalanx) > i and self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if len(self.lphalanx) > i and self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

    def findAndEnableDevices(self):
        # get the time step of the current world.
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

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

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)
        
      
    

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False

        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.time_step = 32
        self.current_angle = 0
        
        self.yolo_model = YOLO("C:/Users/82487/Desktop/my_project2/model/yolov8n.pt")  # 使用上传的模型文件路径
        print("YOLOv8 model loaded successfully.")
        
        self.camera = self.getDevice('CameraTop')
        self.camera.enable(4 * self.timeStep)
    
    def get_image_from_camera(self):
        # 获取摄像头图像
        image = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()

        # 转换图像格式
        image_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        image_rgb = cv2.cvtColor(image_array, cv2.COLOR_BGRA2RGB)
        return image_rgb

    def detect_objects(self):
        image = self.get_image_from_camera()
        resized_image = cv2.resize(image, (640, 640))
        
        results = self.yolo_model(resized_image)
        print(results)
     
        for box in results [0].boxes:
            # 获取类别 ID
                class_id = int(box.cls[0])  # 类别索引
                class_name = self.yolo_model.names[class_id]  # 类别名称
            
            # 检查类别是否为“sports ball”
                if class_name == "sports ball":
                # 获取边界框坐标
                    x_min, y_min, x_max, y_max = box.xyxy[0]  # 边界框坐标
                    box_width = x_max - x_min
                    box_height = y_max - y_min
                    x_center = (x_min + x_max) / 2  # 计算足球的中心点

                print(f"Sports ball detected at center: ({x_center})")
                
                if self.is_close_to_ball(box_width, box_height):
                    print("Close to the ball, stopping.")
                    self.StandInit.play()  # 播放站立初始化动作
                else:
                    print("Moving forward towards the ball.")
                    self.move_towards_ball(x_center)  # 持续前进动作
                break
        
    def move_towards_ball(self, x):
        width = self.camera.getWidth()

        # 判断“sports ball”在图像的中心偏左或偏右
        # if x < width * 0.4:  # 偏左
            # print("Turning left")
            # self.startMotion(self.turnLeft10)
        # elif x > width * 0.6:  # 偏右
            # print("Turning right")
            # self.startMotion(self.turnRight10)
        # else:  # 在图像中心，向前移动
        print("Moving forward")
        self.startMotion(self.forwards50)
            
    def is_close_to_ball(self, width, height):
        # 设置一个阈值来判断是否接近足球，这个值可以根据实际情况调整
        print(f"wh: ({width * height})")
        size_threshold = 8000 # 假设当边界框面积大于此阈值时，表示接近足球
        return width * height >= size_threshold
            
    def run(self):
        self.StandInit.setLoop(True)
        self.StandInit.play()
        self.currentlyPlaying = self.StandInit

        # until a key is pressed
        key = -1
        while robot.step(self.timeStep) != -1:
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
            elif key == Keyboard.LEFT | Keyboard.SHIFT:
                self.startMotion(self.turnLeft60)
            elif key == Keyboard.RIGHT | Keyboard.SHIFT:
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
            elif key == ord('R'):
                self.startMotion(self.Run)
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
                self.detect_objects()
            elif key == ord('3'):
                self.startMotion(self.forwards)
            elif key == ord('4'):
                self.startMotion(self.handWave)
            elif key == ord('5'):
                self.startMotion(self.OpenArms)
            elif key == ord('6'):
                self.startMotion(self.Shake)
            elif key == ord('7'):
                self.setAllLedsColor(0xff0000)  # red
            elif key == ord('8'):
                self.setAllLedsColor(0x00ff00)  # green
            elif key == ord('9'):
                self.setAllLedsColor(0x0000ff)  # blue
            elif key == ord('0'):
                self.StandInit# off
            elif key == ord('H'):
                self.printHelp()

            if robot.step(self.timeStep) == -1:
                break


# create the Robot instance and run main loop
robot = Nao()
robot.run()
