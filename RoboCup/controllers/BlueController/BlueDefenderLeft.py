"""
Blue Team Left Defender robot behaviours.
"""

import os
import sys

# Add the parent directory to the path
current_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import TIME_STEP, Motions


class DefenderLeft(SoccerRobot):
    def run(self):
        self.printSelf()
        count_0 = 0
        flag = 0
        flag1 = 0
        fixedCoordinate = [3.1, -1.2, 0.342]  # 固定的左侧区域位置
        origin = [0.0723, -0.5, 0.0799]  # 起始点的坐标
        goto_Coordinate = [0, 0, 0]

        while self.robot.step(TIME_STEP) != -1:
            if self.isNewBallDataAvailable():
                self.getSupervisorData()
                # Use the ballData (location) to do something.
                data = self.supervisorData
                ballOwner = self.getBallOwner()
                ballCoordinate = self.getBallData()
                blue_fw_l = [data[30], data[31], data[32]]
                blue_fw_r = [data[33], data[34], data[35]]
                redFw = [data[21], data[22], data[23]]
                selfCoordinate = self.getSelfCoordinate()

                # Check if goal was scored
                if self.checkGoal() == 1:
                    self.perform_motion(self.motions.handWave)
                elif self.checkGoal() == -1:
                    self.perform_motion(self.motions.standInit)
                elif self._robot_fell_down(selfCoordinate):
                    self._handle_robot_fall()
                elif self.getBallPriority() == "R":
                    self.perform_motion(self.motions.standInit)
                else:
                    if ballCoordinate[0] >= 2.54 and ballCoordinate[0] <= 4.44:
                        flag = 1
                        if -1.5 <= ballCoordinate[1] <= 0 and (ballOwner == 'BLUE_GK' or ballOwner[0] == 'R'):
                            goto_Coordinate = [4.22, -0.22, 0.315]
                            decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate, blue_fw_l, blue_fw_r, redFw)
                            self._perform_motion(decidedMotion)
                        else:
                            decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate, blue_fw_l, blue_fw_r, redFw)
                            if count_0 >= 2:
                                decidedMotion = self.motions.rightShoot
                                count_0 = 0
                            if decidedMotion == self.motions.longShoot:
                                count_0 += 1
                            self._perform_motion(decidedMotion)
                    else:
                        if (ballCoordinate[0] <= 2.54 or ballCoordinate[0] >= 4.44) and flag == 1:
                            flag1 = 0
                            decidedMotion = self.returnMotion(fixedCoordinate, selfCoordinate)
                            self._perform_motion(decidedMotion)
                            if 2.8 <= selfCoordinate[0] <= 3.2 and -1.0 <= selfCoordinate[1] <= -0.03:
                                flag = 0
                                flag1 = 1
                        if flag1 == 1:
                            decidedMotion = self.turnMotion(origin, selfCoordinate)
                            self._perform_motion(decidedMotion)

            else:
                print("NO BALL DATA!!!")

    def _robot_fell_down(self, selfCoordinate):
        """Check if the robot fell down."""
        return selfCoordinate[2] < 0.2

    def _handle_robot_fall(self):
        """Handle robot recovery after a fall."""
        if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
            decidedMotion = self.motions.standUpFromBack
        else:
            decidedMotion = self.motions.standUpFromFront
        self._perform_motion(decidedMotion)

    def _perform_motion(self, decidedMotion):
        """Perform a given motion if valid."""
        if self.isNewMotionValid(decidedMotion):
            boolean = self.currentlyMoving and (
                self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name
            )
            if boolean:
                self.interruptMotion()
            self.clearMotionQueue()
            if boolean:
                self.addMotionToQueue(self.motions.standInit)
            self.addMotionToQueue(decidedMotion)

        self.startMotion()
