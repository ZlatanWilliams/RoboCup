import math
import numpy as np

class Ball:
    def __init__(self):
        # Ball specifications
        self.circumference = 0.45  # 45cm
        self.diameter = 0.143
        self.radius = self.diameter / 2
        self.mass = 0.31  # kg
        
        # Physics properties
        self.position = np.array([0.0, 0.0], dtype=float) 
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.decay = 0.96
        self.bounce_coefficient = 0.65
        self.owner = None

    def update(self, dt):
        # Air resistance
        speed = math.hypot(*self.velocity)
        if speed > 0:
            drag = 0.5 * 1.225 * 0.38 * (speed ** 2)
            self.velocity = (
                self.velocity[0] - (drag * self.velocity[0]/speed) * dt,
                self.velocity[1] - (drag * self.velocity[1]/speed) * dt
            )
        
        # Update position
        self.position = (
            self.position[0] + self.velocity[0] * dt,
            self.position[1] + self.velocity[1] * dt
        )

class Player:
    def __init__(self, team, role, position):
        self.team = team
        self.role = role
        self.position = position
        self.target_position = position
        self.speed = 2.0  # m/s
        self.kick_power = 5.0
        self.radius = 0.2  # Collision radius

    def update(self, dt):
        dx = self.target_position[0] - self.position[0]
        dy = self.target_position[1] - self.position[1]
        distance = math.hypot(dx, dy)
        
        if distance > 0:
            move_dist = self.speed * dt
            if move_dist > distance:
                self.position = self.target_position
            else:
                self.position = (
                    self.position[0] + (dx/distance) * move_dist,
                    self.position[1] + (dy/distance) * move_dist
                )

class Pitch:
    def __init__(self):
        self.length = 9.0
        self.width = 6.0
        self.goal_width = 2.6
        self.border = 1.0