"""
Red Team Main Controller.
This controller should be selected as controller for Red team robots.
Roles will be assigned automatically for each robot.
"""
from controller import Robot
from RedGoalkeeper import Goalkeeper
from RedDefenderLeft import DefenderLeft
from RedDefenderRight import DefenderRight
from RedForwardLeft import ForwardLeft
from RedForwardRight import ForwardRight



# Create the Robot instance.
robot = Robot()
# Get the Robot Name to find the role.
robotName = robot.getName()

# Compare the Robot Name and assign the role.
if robotName == "RED_GK":
    robotController = Goalkeeper(robot)
elif robotName == "RED_DEF_L":
    robotController = DefenderLeft(robot)
elif robotName == "RED_DEF_R":
    robotController = DefenderRight(robot)
elif robotName == "RED_FW_L":
    robotController = ForwardLeft(robot)
else:
    robotController = ForwardRight(robot)

# Run the Robot Controller.
robotController.run()