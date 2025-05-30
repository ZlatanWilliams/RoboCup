"""
The Basic Supervisor class.
All Supervisor classes should be derived from this class.
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from controller import Supervisor
import struct
from Utils.Consts import (TIME_STEP)
from Utils import Functions

class SupervisorBase(Supervisor):
  def __init__(self):
    super().__init__()

    self.emitter = self.getDevice("emitter")

    self.ball = self.getFromDef("SOCCERBALL")

    self.robots = {
      "RED_GK"    : self.getFromDef("RED_GK"),
      "RED_DEF_L" : self.getFromDef("RED_DEF_L"),
      "RED_DEF_R" : self.getFromDef("RED_DEF_R"),
      "RED_FW"  : self.getFromDef("RED_FW"),
      "BLUE_GK"   : self.getFromDef("BLUE_GK"),
      "BLUE_DEF_L": self.getFromDef("BLUE_DEF_L"),
      "BLUE_DEF_R": self.getFromDef("BLUE_DEF_R"),
      "BLUE_FW_L" : self.getFromDef("BLUE_FW_L"),
    }

    self.ballPriority = "R"
    self.previousBallLocation = [0, 0, 0.0798759]
  
  def getBallPosition(self) -> list:
    """Get the soccer ball coordinate on the field.

    Returns:
        list: x, y, z coordinates.
    """
    ballTranslation = self.ball.getField("translation")
    newBallLocation = ballTranslation.getSFVec3f()

    if abs(newBallLocation[0]) < 4.5 and abs(newBallLocation[1]) < 3:
      if (self.previousBallLocation[0] + 0.05 < newBallLocation[0] or self.previousBallLocation[0] - 0.05 > newBallLocation[0] or\
          self.previousBallLocation[1] + 0.05 < newBallLocation[1] or self.previousBallLocation[1] - 0.05 > newBallLocation[1]):
        self.ballPriority = "N"
        self.previousBallLocation = newBallLocation


    return newBallLocation
  
  def setBallPosition(self, ballPosition) -> None:
    """Set the soccer ball coordinate on the field.

    Args:
        list: x, y, z coordinates.
    """
    self.previousBallLocation = ballPosition
    ballTranslation = self.ball.getField("translation")
    ballTranslation.setSFVec3f(ballPosition)
    self.ball.resetPhysics()

  def getRobotPosition(self, robotName) -> list:
    """Get the robot coordinate on the field.

    Returns:
        list: x, y, z coordinates.
    """
    robotTranslation = self.robots[robotName].getField("translation")
    return robotTranslation.getSFVec3f()

  def getBallOwner(self) -> str:
    """Calculate the ball owner team from the distances from the ball.

    Returns:
        str: Ball owner team first letter.
    """

    ballPosition = self.getBallPosition()
    ballOwnerRobotName = "RED_GK"
    minDistance = Functions.calculateDistance(ballPosition, self.getRobotPosition(ballOwnerRobotName))
    for i, key in enumerate(self.robots):
      tempDistance = Functions.calculateDistance(ballPosition, self.getRobotPosition(key))
      if tempDistance < minDistance:
        minDistance = tempDistance
        ballOwnerRobotName = key

    if len(ballOwnerRobotName) < 9:
      for i in range(len(ballOwnerRobotName), 9):
        ballOwnerRobotName = ballOwnerRobotName + '*'

    return ballOwnerRobotName

  def sendSupervisorData(self) -> None:
    """Send Data (ballPosition, ballOwner, ballPriority, ...) to Robots. Channel is '0'."""
    
    ballPosition = self.getBallPosition()
    ballOwner = bytes(self.getBallOwner(), 'utf-8')
    ballPriority = bytes(self.ballPriority, 'utf-8')

    redGk = self.getRobotPosition("RED_GK")
    redDefLeft = self.getRobotPosition("RED_DEF_L")
    redDefRight = self.getRobotPosition("RED_DEF_R")
    redFw = self.getRobotPosition("RED_FW")
    blueGk = self.getRobotPosition("BLUE_GK")
    blueDefLeft = self.getRobotPosition("BLUE_DEF_L")
    blueDefRight = self.getRobotPosition("BLUE_DEF_R")
    blueFwLeft = self.getRobotPosition("BLUE_FW_L")

    data = struct.pack('dd9ss24d', ballPosition[0], ballPosition[1], ballOwner, ballPriority, redGk[0], redGk[1], redGk[2], redDefLeft[0], redDefLeft[1], redDefLeft[2], redDefRight[0], redDefRight[1], redDefRight[2],\
                                  redFw[0], redFw[1], redFw[2], blueGk[0], blueGk[1], blueGk[2], blueDefLeft[0], blueDefLeft[1], blueDefLeft[2],  # 修改部分
                               blueDefRight[0], blueDefRight[1], blueDefRight[2], blueFwLeft[0], blueFwLeft[1], blueFwLeft[2])
    self.emitter.send(data)

  def setBallPriority(self, priority):
    self.ballPriority = priority

  def resetSimulation(self):
    self.previousBallLocation = [0, 0, 0.0798759]
    self.simulationReset()
    for robot in self.robots.values():
      robot.resetPhysics()
  
  def stopSimulation(self):
    self.simulationSetMode(self.SIMULATION_MODE_PAUSE)
