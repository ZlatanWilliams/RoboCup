"""
Red Team Right Defender robot behaviours.
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import (TIME_STEP, Motions)
from Utils import Functions
import RedTeamStrategies

class DefenderRight (SoccerRobot):
  def run(self):

    while self.robot.step(TIME_STEP) != -1:

      if self.isNewBallDataAvailable():

        self.getSupervisorData()

        # Use the ballData (location) to do something.
        ballCoordinate = self.getBallData()
        # print("RedDefenderRight - ballCoordinate: ", ballCoordinate)
        selfCoordinate = self.getSelfCoordinate()
        # print("RedDefenderRight - selfCoordinate: ", selfCoordinate)
        decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)
        # print("RedDefenderRight - decidedMotion: ", decidedMotion.Name)
        if self.isNewMotionValid(decidedMotion):
          forwardsSprintInterrupt = self.currentlyMoving and (self.currentlyMoving.name == self.motions.forwardsSprint.name and decidedMotion.name != self.motions.forwardsSprint.name)

          # interruptCheck = self.currentlyMoving and\
          #          (self.currentlyMoving.name == self.motions.turnLeft40.name and decidedMotion.name != self.motions.turnLeft40.name and\
          #           decidedMotion.name != self.motions.sideStepLeft.name and decidedMotion.name != self.motions.sideStepRight.name) or\
          #          (self.currentlyMoving.name == self.motions.turnRight40.name and decidedMotion.name != self.motions.turnRight40.name)

          leftShootCheck = self.currentlyMoving and self.currentlyMoving.name == self.motions.rightShoot.name and self.currentlyMoving.isOver() and decidedMotion.name == self.motions.rightShoot.name

          # if interruptCheck:
          #   self.interruptMotion()
          # if forwardsSprintInterrupt:
          #   self.interruptForwardsSprint()
            # print("RedDefenderRight - Motion interrupted!")
          self.clearMotionQueue()
          # if interruptCheck:
          #   self.addMotionToQueue(self.motions.standInit)
          if leftShootCheck:
            self.addMotionToQueue(self.motions.shoot)
          else:
            self.addMotionToQueue(decidedMotion)
        
        self.startMotion()
      else:

        # It seems there is a problem.
        # print("NO BALL DATA!!!")
        pass

  # Override decideMotion
  def decideMotion(self, ballCoordinate, selfCoordinate):

    # Check the goal scored to balance itself.
    if self.checkGoal() == 1:
      return self.motions.handWave
    elif self.checkGoal() == -1:
      return self.motions.standInit

    # Fall Detection
    robotHeightFromGround = self.getSelfCoordinate()[2]
    if robotHeightFromGround < 0.2:
      if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
        return self.motions.standUpFromBack
      else:
        return self.motions.standUpFromFront

    # Check the oponent has ball priority.
    if self.getBallPriority() == "B":
      return self.motions.standInit

    # We are going to use these values to check if there is an obstacle in front of the robot.
    leftDistance = self.getLeftSonarValue()
    rightDistance = self.getRightSonarValue()

    robotHeadingAngle = self.getRollPitchYaw()[2]

    # If the ball on the opponent field.
    if RedTeamStrategies.getZone(ballCoordinate) > 9:

      # If the ball on team member.
      if self.getBallOwner()[0] == "R" and self.getBallOwner() != "RED_DEF_R":

        # Go to zone 15.
        if RedTeamStrategies.getZone(selfCoordinate) != 15:
          zoneTargetX = (RedTeamStrategies.PLAY_ZONE[15][0][0] + RedTeamStrategies.PLAY_ZONE[15][1][0]) / 2
          zoneTargetY = RedTeamStrategies.PLAY_ZONE[15][0][1]
          # Find the angle between the target zone and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([zoneTargetX, zoneTargetY], selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion
            
          # Check if there is an obstacle in front of the robot.
          if self.obstacleAvoidance:
            if leftDistance < 0.75:
              return self.motions.sideStepRight
            elif rightDistance < 0.75:
              return self.motions.sideStepLeft
        
          if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
            self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

          return self.motions.forwardsSprint

        # Head to ball.
        else:
          # print("I am waiting on 15")
          # Find the angle between the ball and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion

      # If the ball on the opponent.
      elif self.getBallOwner()[0] == "B":
        
        # Go to zone 6.
        if RedTeamStrategies.getZone(selfCoordinate) != 6:
          # print("I am going to 6")
          # Bottom line of zone 6.
          zoneTargetX = (RedTeamStrategies.PLAY_ZONE[6][0][0] + RedTeamStrategies.PLAY_ZONE[6][1][0]) / 2
          zoneTargetY = RedTeamStrategies.PLAY_ZONE[6][0][1]
          # Find the angle between the target zone and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([zoneTargetX, zoneTargetY], selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion
            
          # Check if there is an obstacle in front of the robot.
          if self.obstacleAvoidance:
            if leftDistance < 0.75:
              return self.motions.sideStepRight
            elif rightDistance < 0.75:
              return self.motions.sideStepLeft
        
          if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
            self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

          return self.motions.forwardsSprint

        # Head to ball.
        else:
          # print("I am waiting on 6")
          # Find the angle between the ball and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion

       # The ball on the robot itself
      else:
        # print("I am going to press")
        # Find the angle between the ball and robot heading.
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
        turningMotion = self.getTurningMotion(turningAngle)
        if turningMotion is not None:
          return turningMotion

        bodyDistanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

        # Decide wehere to shoot or pass.
        if bodyDistanceFromBall < 0.25:
          # We have to look for the distance from left foot becuase or robots are left footed.
          
          # If the robot at the 16th or 18th zones, the goal is 17th zone.
          if RedTeamStrategies.getZone(ballCoordinate) == 16 or RedTeamStrategies.getZone(ballCoordinate) == 18:
            turningAngleForGoalLeft = Functions.calculateTurningAngleAccordingToRobotHeading([RedTeamStrategies.PLAY_ZONE[17][0][0], 0], selfCoordinate, robotHeadingAngle)
            turningAngleForGoalRight = Functions.calculateTurningAngleAccordingToRobotHeading([RedTeamStrategies.PLAY_ZONE[17][1][0], 0], selfCoordinate, robotHeadingAngle)

          # Else, shoot to goal!
          else:
            # If decided to shoot
            # We have to calculate the goal angle and sideSteps according to this angle.
            turningAngleForGoalLeft = Functions.calculateTurningAngleAccordingToRobotHeading(RedTeamStrategies.BLUE_GOAL["Left"], selfCoordinate, robotHeadingAngle)
            turningAngleForGoalRight = Functions.calculateTurningAngleAccordingToRobotHeading(RedTeamStrategies.BLUE_GOAL["Right"], selfCoordinate, robotHeadingAngle)
          
          if (turningAngleForGoalLeft > 0 and turningAngleForGoalRight > 0):
            if turningAngleForGoalLeft < 76 or turningAngleForGoalRight < 76:
              return self.motions.rightSidePass
            return self.motions.sideStepRight
          elif (turningAngleForGoalLeft < 0 and turningAngleForGoalRight < 0):
            if turningAngleForGoalLeft > -76 or turningAngleForGoalRight > -76:
              return self.motions.leftSidePass
            return self.motions.sideStepLeft
          elif (abs(turningAngleForGoalLeft) > 90 and abs(turningAngleForGoalRight) > 90):
            if abs(turningAngleForGoalLeft) > abs(turningAngleForGoalRight):
              return self.motions.sideStepLeft
            else:
              return self.motions.sideStepRight
          else:
            if bodyDistanceFromBall < 0.2:
              return self.motions.rightShoot
            else:
              return self.motions.forwardsSprint
        
        # Check if there is an obstacle in front of the robot.
        elif self.obstacleAvoidance and leftDistance < bodyDistanceFromBall:
          if leftDistance < 0.5:
            return self.motions.sideStepRight
          elif rightDistance < 0.5:
            return self.motions.sideStepLeft
        
        if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
          self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

        return self.motions.forwardsSprint

    # The ball on right or middle zone field.
    elif RedTeamStrategies.getZone(ballCoordinate) == 3 or RedTeamStrategies.getZone(ballCoordinate) == 6 or RedTeamStrategies.getZone(ballCoordinate) == 9 or \
         RedTeamStrategies.getZone(ballCoordinate) == 2 or RedTeamStrategies.getZone(ballCoordinate) == 5 or RedTeamStrategies.getZone(ballCoordinate) == 8:

      # If the ball on team member.
      if self.getBallOwner()[0] == "R" and self.getBallOwner() != "RED_DEF_R" and self.getBallOwner() != "RED_GK":
        
        # Go to zone 9.
        if RedTeamStrategies.getZone(selfCoordinate) != 9:
          # print("I am going to 9")
          # Bottom line of zone 9.
          zoneTargetX = (RedTeamStrategies.PLAY_ZONE[9][0][0] + RedTeamStrategies.PLAY_ZONE[9][1][0]) / 2
          zoneTargetY = RedTeamStrategies.PLAY_ZONE[9][0][1]
          # Find the angle between the target zone and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([zoneTargetX, zoneTargetY], selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion
            
          # Check if there is an obstacle in front of the robot.
          if self.obstacleAvoidance:
            if leftDistance < 0.75:
              return self.motions.sideStepRight
            elif rightDistance < 0.75:
              return self.motions.sideStepLeft
        
          if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
            self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

          return self.motions.forwardsSprint

        # Head to ball.
        else:
          # print("I am waiting on 9")
          # Find the angle between the ball and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion
        
      # The ball on the opponent or on the robot itself.
      else:
        # print("I am going to press")
        # Find the angle between the ball and robot heading.
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
        turningMotion = self.getTurningMotion(turningAngle)
        if turningMotion is not None:
          return turningMotion

        bodyDistanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

        # Decide wehere to shoot or pass.
        if bodyDistanceFromBall < 0.25:
          # We have to look for the distance from left foot becuase or robots are left footed.
          
          # The goal is the 11th zone. Passing...
          turningAngleForGoalLeft = Functions.calculateTurningAngleAccordingToRobotHeading([RedTeamStrategies.PLAY_ZONE[11][0][1], RedTeamStrategies.PLAY_ZONE[11][1][0]], selfCoordinate, robotHeadingAngle)
          turningAngleForGoalRight = Functions.calculateTurningAngleAccordingToRobotHeading([RedTeamStrategies.PLAY_ZONE[11][0][0], RedTeamStrategies.PLAY_ZONE[11][1][1]], selfCoordinate, robotHeadingAngle)
          
          if (turningAngleForGoalLeft > 0 and turningAngleForGoalRight > 0):
            if turningAngleForGoalLeft < 76 or turningAngleForGoalRight < 76:
              return self.motions.rightSidePass
            return self.motions.sideStepRight
          elif (turningAngleForGoalLeft < 0 and turningAngleForGoalRight < 0):
            if turningAngleForGoalLeft > -76 or turningAngleForGoalRight > -76:
              return self.motions.leftSidePass
            return self.motions.sideStepLeft
          elif (abs(turningAngleForGoalLeft) > 90 and abs(turningAngleForGoalRight) > 90):
            if abs(turningAngleForGoalLeft) > abs(turningAngleForGoalRight):
              return self.motions.sideStepLeft
            else:
              return self.motions.sideStepRight
          else:
            if bodyDistanceFromBall < 0.2:
              return self.motions.rightShoot
            else:
              return self.motions.forwardsSprint
        
        # Check if there is an obstacle in front of the robot.
        elif self.obstacleAvoidance and leftDistance < bodyDistanceFromBall:
          if leftDistance < 0.5:
            return self.motions.sideStepRight
          elif rightDistance < 0.5:
            return self.motions.sideStepLeft
        
        if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
          self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

        return self.motions.forwardsSprint

    # The ball on right zone.
    else:
      # If the ball on team member.
      if self.getBallOwner()[0] == "R" and self.getBallOwner() != "RED_DEF_R" and self.getBallOwner() != "RED_GK":
        
        # Go to zone 9.
        if RedTeamStrategies.getZone(selfCoordinate) != 9:
          # print("I am going to 9")
          # Bottom line of zone 9.
          zoneTargetX = (RedTeamStrategies.PLAY_ZONE[9][0][0] + RedTeamStrategies.PLAY_ZONE[9][1][0]) / 2
          zoneTargetY = RedTeamStrategies.PLAY_ZONE[9][0][1]
          # Find the angle between the target zone and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([zoneTargetX, zoneTargetY], selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion
            
          # Check if there is an obstacle in front of the robot.
          if self.obstacleAvoidance:
            if leftDistance < 0.75:
              return self.motions.sideStepRight
            elif rightDistance < 0.75:
              return self.motions.sideStepLeft
        
          if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
            self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

          return self.motions.forwardsSprint

        # Head to ball.
        else:
          # print("I am waiting on 9")
          # Find the angle between the ball and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion
        
      # The ball on the opponent.
      else:
        # Go to zone 6.
        if RedTeamStrategies.getZone(selfCoordinate) != 6:
          # print("I am going to 6")
          # Bottom line of zone 6.
          zoneTargetX = (RedTeamStrategies.PLAY_ZONE[6][0][0] + RedTeamStrategies.PLAY_ZONE[6][1][0]) / 2
          zoneTargetY = RedTeamStrategies.PLAY_ZONE[6][0][1]
          # Find the angle between the target zone and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([zoneTargetX, zoneTargetY], selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion
            
          # Check if there is an obstacle in front of the robot.
          if self.obstacleAvoidance:
            if leftDistance < 0.75:
              return self.motions.sideStepRight
            elif rightDistance < 0.75:
              return self.motions.sideStepLeft
        
          if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:  # we reached the end of forward.motion
            self.currentlyMoving.setTime(360)  # loop back to the beginning of the walking sequence

          return self.motions.forwardsSprint

        # Head to ball.
        else:
          # print("I am waiting on 6")
          # Find the angle between the ball and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion

    # Stand by.
    return self.motions.standInit