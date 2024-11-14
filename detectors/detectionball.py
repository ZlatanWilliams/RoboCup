# 导入必要的库 
from controller import Robot, Keyboard, Motion
import math
import cv2
import numpy as np
from ultralytics import YOLO
import time

# 定义PID控制器类
class PIDController:
    """用于平滑头部运动的简单PID控制器"""
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

# 定义主要的Nao机器人类
class Nao(Robot):
    PHALANX_MAX = 8

    # 加载运动文件
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
        # 注释掉无法找到的运动文件
        # self.Shake = Motion('../../motions/ShakeHead.motion')
        # self.Run = Motion('../../motions/Run.motion')

    def startMotion(self, motion):
        # 中断当前动作
        if self.currentlyPlaying is not None and not self.currentlyPlaying.isOver():
            self.currentlyPlaying.stop()
        # 开始新动作
        motion.play()
        self.currentlyPlaying = motion

    def printHelp(self):
        print('----------nao_demo----------')
        print('使用键盘来控制机器人（一次只能一个动作）')
        print('(需要聚焦于3D窗口)')
        print('[Up][Down]: 前进/后退一步')
        print('[<-][->]: 左/右侧步')
        print('[Shift] + [<-][->]: 左/右转')
        print('[U]: 打印超声波传感器信息')
        print('[A]: 打印加速度计信息')
        print('[G]: 打印陀螺仪信息')
        print('[S]: 打印GPS信息')
        print('[I]: 打印惯性单元信息（滚转/俯仰/偏航）')
        print('[F]: 打印足部传感器信息')
        print('[B]: 打印足部碰撞传感器信息')
        print('[Home][End]: 打印缩放后的顶部/底部摄像头图像')
        print('[PageUp][PageDown]: 张开/握紧双手')
        print('[7][8][9]: 改变所有LED的RGB颜色')
        print('[0]: 关闭所有LED')
        print('[T]: 执行 SidePass_Right 动作')
        print('[W]: 擦拭额头')
        print('[K]: 执行 Shoot 动作')
        print('[P]: 执行 LongPass 动作')
        print('[N]: 从背部站起来')
        print('[M]: 从正面站起来')
        # print('[R]: 执行 Run 动作')  # 已注释，因为运动文件缺失
        print('[1]: 执行 RightShoot 动作')
        print('[2]: 执行 ForwardsSprint 动作')
        print('[3]: 执行 Forwards 动作')
        print('[4]: 执行 HandWave 动作')
        print('[5]: 执行 OpenArms 动作')
        # print('[6]: 执行 ShakeHead 动作')  # 已注释，因为运动文件缺失
        print('[H]: 打印此帮助信息')
        print('[L]: 开启/关闭球跟踪')

    def findAndEnableDevices(self):
        # 获取当前世界的时间步长
        self.timeStep = int(self.getBasicTimeStep())

        # 头部电机
        self.head_yaw = self.getDevice('HeadYaw')
        self.head_pitch = self.getDevice('HeadPitch')

        # 启用头部电机的位置传感器
        self.head_yaw_sensor = self.head_yaw.getPositionSensor()
        self.head_yaw_sensor.enable(self.timeStep)

        self.head_pitch_sensor = self.head_pitch.getPositionSensor()
        self.head_pitch_sensor.enable(self.timeStep)

        # 摄像头
        self.cameraTop = self.getDevice('CameraTop')
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom = self.getDevice('CameraBottom')
        self.cameraBottom.enable(4 * self.timeStep)

        self.width = self.cameraTop.getWidth()
        self.height = self.cameraTop.getHeight()
        print(f"Camera dimensions: {self.width} x {self.height}")

        # 其他传感器和设备
        # 加速度计
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # 陀螺仪
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(4 * self.timeStep)

        # GPS
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timeStep)

        # 惯性单元
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # 超声波传感器
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # 足部传感器
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # 足部碰撞传感器
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

        # 手指电机
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # 假设左右手具有相同的电机位置范围
            if self.rphalanx[i] is not None:
                self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
                self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())
            else:
                self.maxPhalanxMotorPosition.append(0.0)
                self.minPhalanxMotorPosition.append(0.0)

        # 肩部电机
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # 键盘控制
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = None
        self.tracking = False  # 初始时跟踪功能关闭
        self.rotating = False  # 旋转状态

        # 初始化设备和变量
        self.findAndEnableDevices()
        self.loadMotionFiles()
        self.printHelp()

        # 初始化头部运动的 PID 控制器
        self.pid_yaw = PIDController(kp=0.002, ki=0.0, kd=0.001)
        self.pid_pitch = PIDController(kp=0.002, ki=0.0, kd=0.001)

        # 加载 YOLO 模型
        # 如果需要，替换为模型文件的绝对路径
        self.yolo_model = YOLO('yolov8n.pt')  # 确保模型可用

        # 打印模型类别名称，便于调试
        self.class_names = self.yolo_model.model.names
        print("Model class names:", self.class_names)

        # 获取足球的类别 ID
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
        # 开始执行 StandInit 动作
        self.StandInit.setLoop(True)
        self.StandInit.play()
        self.currentlyPlaying = self.StandInit

        # 设置最小置信度阈值
        MIN_CONFIDENCE = 0.15  # 根据需要调整

        while self.step(self.timeStep) != -1:
            # 处理键盘输入
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
                # elif key == ord('R'):
                #     self.startMotion(self.Run)  # 已注释，因为运动文件缺失
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
                # elif key == ord('6'):
                #     self.startMotion(self.Shake)  # 已注释，因为运动文件缺失
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
                # 获取顶部摄像头图像
                image_top = self.cameraTop.getImage()
                image_top = np.frombuffer(image_top, np.uint8).reshape((self.height, self.width, 4))
                image_top = cv2.cvtColor(image_top, cv2.COLOR_BGRA2BGR)

                # 获取底部摄像头图像
                image_bottom = self.cameraBottom.getImage()
                image_bottom = np.frombuffer(image_bottom, np.uint8).reshape((self.height, self.width, 4))
                image_bottom = cv2.cvtColor(image_bottom, cv2.COLOR_BGRA2BGR)

                # 可选：显示摄像头图像（调试用）
                # cv2.imshow("Top Camera", image_top)
                # cv2.imshow("Bottom Camera", image_bottom)
                # cv2.waitKey(1)

                # 运行 YOLO 检测
                results_top = self.yolo_model(image_top, conf=MIN_CONFIDENCE)
                results_bottom = self.yolo_model(image_bottom, conf=MIN_CONFIDENCE)

                # 初始化变量
                ball_detected = False
                ball_x = 0
                ball_y = 0
                camera_used = 'top'

                # 检测顶部摄像头
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

                # 如果顶部未检测到，检测底部摄像头
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
                    # 停止旋转
                    if self.rotating:
                        self.rotating = False
                        self.StandInit.play()
                        self.currentlyPlaying = self.StandInit
                        print("Stopping rotation as ball is detected.")

                    # 计算球的位置与图像中心的偏差
                    error_x = (self.width / 2) - ball_x
                    error_y = (self.height / 2) - ball_y

                    # 更新 PID 控制器
                    yaw_adjust = self.pid_yaw.update(error_x)
                    pitch_adjust = self.pid_pitch.update(error_y)

                    # 获取当前头部位置
                    current_yaw = self.head_yaw_sensor.getValue()
                    current_pitch = self.head_pitch_sensor.getValue()

                    # 计算新的位置
                    new_yaw = current_yaw + yaw_adjust
                    new_pitch = current_pitch - pitch_adjust  # 由于图像坐标，需要减去

                    # 根据球的纵向位置调整头部
                    norm_y = ball_y / self.height
                    if camera_used == 'bottom':
                        if norm_y > 0.5:
                            new_pitch -= 0.05  # 向下看
                            print("Adjusting head pitch downward.")
                        elif norm_y < 0.2:
                            new_pitch += 0.05  # 向上看
                            print("Adjusting head pitch upward.")

                    # 限制在电机的范围内
                    new_yaw = max(min(new_yaw, self.head_yaw.getMaxPosition()), self.head_yaw.getMinPosition())
                    new_pitch = max(min(new_pitch, self.head_pitch.getMaxPosition()), self.head_pitch.getMinPosition())

                    # 设置新的头部位置
                    self.head_yaw.setPosition(new_yaw)
                    self.head_pitch.setPosition(new_pitch)

                    print(f"Ball detected by {camera_used} camera at ({ball_x:.2f}, {ball_y:.2f}). Adjusting head to ({new_yaw:.2f}, {new_pitch:.2f}).")

                    # 根据球的位置切换摄像头
                    if camera_used == 'top' and norm_y > 0.8:
                        self.active_camera = 'bottom'
                        print("Switching to bottom camera")
                    elif camera_used == 'bottom' and norm_y < 0.2:
                        self.active_camera = 'top'
                        print("Switching to top camera")

                    # 移动机器人朝向球的方向
                    if not self.currentlyPlaying == self.forwards:
                        self.startMotion(self.forwards)
                        print("Moving forwards towards the ball.")
                else:
                    print("Ball not detected by any camera.")
                    # 利用超声波传感器辅助定位
                    # 获取超声波传感器的距离信息
                    if len(self.us) >= 2:
                        left_dist = self.us[0].getValue()
                        right_dist = self.us[1].getValue()
                    else:
                        left_dist, right_dist = (None, None)

                    print('-----ultrasonic sensors-----')
                    print(f'left: {left_dist:.2f} m, right: {right_dist:.2f} m')

                    # 定义距离阈值
                    FRONT_THRESHOLD = 1.0  # 1 meter

                    if left_dist is not None and right_dist is not None:
                        if left_dist < FRONT_THRESHOLD and right_dist < FRONT_THRESHOLD:
                            # 球在前方，移动前进
                            if not self.rotating or (self.currentlyPlaying != self.forwards):
                                self.startMotion(self.forwards)
                                self.rotating = False
                                print("Moving forwards towards the ball based on ultrasonic sensors.")
                        elif left_dist < FRONT_THRESHOLD:
                            # 球在左侧，转右
                            self.startMotion(self.turnRight60)
                            self.rotating = False
                            print("Turning right towards the ball based on ultrasonic sensors.")
                        elif right_dist < FRONT_THRESHOLD:
                            # 球在右侧, 转左
                            self.startMotion(self.turnLeft60)
                            self.rotating = False
                            print("Turning left towards the ball based on ultrasonic sensors.")
                        else:
                            # 球在后方, 转180度
                            # 由于没有180度的动作文件，这里连续播放两次 turnLeft60
                            self.startMotion(self.turnLeft60)
                            self.rotating = False
                            print("Turning left to search for the ball behind based on ultrasonic sensors.")
                    elif left_dist is not None:
                        if left_dist < FRONT_THRESHOLD:
                            self.startMotion(self.turnRight60)
                            self.rotating = False
                            print("Turning right towards the ball based on left ultrasonic sensor.")
                        else:
                            # 未检测到，继续旋转
                            pass
                    elif right_dist is not None:
                        if right_dist < FRONT_THRESHOLD:
                            self.startMotion(self.turnLeft60)
                            self.rotating = False
                            print("Turning left towards the ball based on right ultrasonic sensor.")
                        else:
                            # 未检测到，继续旋转
                            pass
                    else:
                        # 无传感器数据, 继续旋转
                        pass

                    # 如果机器人未在旋转状态，开始旋转
                    if not self.rotating:
                        print("Starting rotation to find the ball.")
                        self.rotating = True
                        self.turnLeft60.play()
                        self.currentlyPlaying = self.turnLeft60
                    else:
                        # 检查旋转动作是否完成
                        if self.currentlyPlaying.isOver():
                            print("Rotation motion over, restarting rotation.")
                            self.turnLeft60.play()
                            self.currentlyPlaying = self.turnLeft60

            # 可选：添加一个小的延迟，以防止过度使用 CPU
            time.sleep(0.01)
        else:
            # 未启用跟踪时的其他行为
            pass

    # 实现其他方法，按照您的原始代码

    def useUltrasonicSensors(self):
        # 获取超声波传感器的距离信息
        distances = [sensor.getValue() for sensor in self.us]
        left_dist, right_dist = distances if len(distances) >= 2 else (None, None)

        print('-----ultrasonic sensors-----')
        print(f'left: {left_dist:.2f} m, right: {right_dist:.2f} m')

        # 根据超声波传感器信息进行简单导航（示例）
        if left_dist is not None and right_dist is not None:
            if left_dist < right_dist:
                # 如果左侧距离较近，向右转
                self.startMotion(self.turnRight60)
                print("Turning right towards the ball based on ultrasonic sensors.")
            elif right_dist < left_dist:
                # 如果右侧距离较近，向左转
                self.startMotion(self.turnLeft60)
                print("Turning left towards the ball based on ultrasonic sensors.")
            else:
                # 如果距离相近，继续前进
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
        # 实现您的原始代码
        pass

    def printFootBumpers(self):
        # 实现您的原始代码
        pass

    def printUltrasoundSensors(self):
        dist = []
        for sensor in self.us:
            dist.append(sensor.getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right: %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # 定义图像缩小的比例
        width = camera.getWidth()
        height = camera.getHeight()

        # 读取摄像头的 RGB 像素值
        image = camera.getImage()

        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        for y in range(0, int(height / scaled)):
            line = ''
            for x in range(0, int(width / scaled)):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255  # 重缩放到 0 到 9
                line = line + str(int(gray))
            print(line)

    def setAllLedsColor(self, rgb):
        # 这些 LED 接受 RGB 值
        for i in range(0, len(self.leds)):
            self.leds[i].set(rgb)

        # 耳朵的 LED 是单色（蓝色）
        # 接受 0 - 255 之间的值
        self.leds[5].set(rgb & 0xFF)
        self.leds[6].set(rgb & 0xFF)

    def setHandsAngle(self, angle):
        for i in range(0, self.PHALANX_MAX):
            # 获取电机的最小和最大位置
            minPosition = self.minPhalanxMotorPosition[i]
            maxPosition = self.maxPhalanxMotorPosition[i]

            # 确保最小位置非负
            if minPosition < 0:
                minPosition = 0.0

            # 将角度限制在最小和最大位置之间
            clampedAngle = max(min(angle, maxPosition), minPosition)

            if self.rphalanx[i] is not None:
                self.rphalanx[i].setPosition(clampedAngle)
            if self.lphalanx[i] is not None:
                self.lphalanx[i].setPosition(clampedAngle)

# 创建机器人实例并运行主循环
robot = Nao()
robot.run()
