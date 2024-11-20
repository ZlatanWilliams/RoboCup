"""
Red Team Right Forward robot behaviours.
"""

import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import (TIME_STEP, Motions)
from Utils import Functions
import RedTeamStrategies

class ForwardRight(SoccerRobot):
    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            if self.isNewBallDataAvailable():
                # 获取监督者数据
                self.getSupervisorData()

                # 获取球的位置和机器人的位置
                ballCoordinate = self.getBallData()
                selfCoordinate = self.getSelfCoordinate()

                # 决定动作
                decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)

                if self.isNewMotionValid(decidedMotion):
                    # 管理当前动作
                    rightShootCheck = self.currentlyMoving and self.currentlyMoving.name == self.motions.rightShoot.name and self.currentlyMoving.isOver() and decidedMotion.name == self.motions.rightShoot.name
                    self.clearMotionQueue()
                    if rightShootCheck:
                        self.addMotionToQueue(self.motions.shoot)
                    else:
                        self.addMotionToQueue(decidedMotion)

                self.startMotion()
            else:
                # 没有球数据
                pass

    # Override decideMotion
    def decideMotion(self, ballCoordinate, selfCoordinate):
        # 检查进球状态
        if self.checkGoal() == 1:
            return self.motions.handWave
        elif self.checkGoal() == -1:
            return self.motions.standInit

        # 检查机器人是否跌倒
        robotHeightFromGround = self.getSelfCoordinate()[2]
        if robotHeightFromGround < 0.2:
            if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
                return self.motions.standUpFromBack
            else:
                return self.motions.standUpFromFront

        # 如果对方控制球
        if self.getBallPriority() == "B":
            return self.motions.standInit

        robotHeadingAngle = self.getRollPitchYaw()[2]
        bodyDistanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

        # 区分右前锋的行为
        if bodyDistanceFromBall < 0.25:
            turningAngleForGoal = Functions.calculateTurningAngleAccordingToRobotHeading(RedTeamStrategies.BLUE_GOAL["Left"], selfCoordinate, robotHeadingAngle)
            if abs(turningAngleForGoal) < 90:
                return self.motions.leftSidePass
            else:
                return self.motions.rightShoot

        # 寻找球的方向
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
        turningMotion = self.getTurningMotion(turningAngle)
        if turningMotion is not None:
            return turningMotion

        return self.motions.forwardsSprint
