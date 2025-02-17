import pygame
import numpy as np

class Ball:
    def __init__(self):
        self.position = np.array([0.0, 0.0], dtype=float) 
        self.velocity = np.array([0.0, 0.0], dtype=float)
        
        self.radius = 0.1  # meters
        self.color = (255, 255, 0)
        self.panel_color = (0, 0, 0)
        
        self.mass = 0.45
        self.friction = 0.98
        self.bounce_damping = 0.8
        self.max_speed = 10.0
        self.owner = None

    def update(self, dt, pitch_width, pitch_length):
        """Update ball position based on physics"""
        # Update position based on velocity
        print(f"Debug: dt = {dt}, type(dt) = {type(dt)}") 

        if not isinstance(dt, (float, int)):
            raise TypeError(f"Invalid dt type: {type(dt)}. Expected float or int.")
        self.position += np.asarray(self.velocity) * dt
        
        # Apply friction
        self.velocity *= np.asarray(self.friction)
        
        # Stop if velocity is very small
        if np.linalg.norm(self.velocity) < 0.01:
            self.velocity = np.array([0.0, 0.0])
        
        # Handle collisions with pitch boundaries
        self._handle_boundary_collision(pitch_width, pitch_length)

    def _handle_boundary_collision(self, pitch_width, pitch_length):
        """Handle collisions with pitch boundaries"""
        # Check horizontal boundaries (including goals)
        if abs(self.position[0]) > pitch_length/2:
            # Ball hit the end of the pitch
            self.position[0] = np.sign(self.position[0]) * pitch_length/2
            self.velocity[0] *= -self.bounce_damping
        
        # Check vertical boundaries
        if abs(self.position[1]) > pitch_width/2:
            self.position[1] = np.sign(self.position[1]) * pitch_width/2
            self.velocity[1] *= -self.bounce_damping

    def apply_force(self, force):
        """Apply a force to the ball"""
        acceleration = force / self.mass
        self.velocity += acceleration
        
        # Limit speed
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = (self.velocity / speed) * self.max_speed

    def kick(self, direction, power):
        """Kick the ball in a given direction with given power"""
        # Normalize direction vector
        direction = direction / np.linalg.norm(direction)
        
        # Apply force based on power (power should be between 0 and 1)
        force = direction * (power * 50)  # 50 is maximum kick force
        self.apply_force(force)

    def get_state(self):
        """Get current state of the ball"""
        return {
            'position': self.position,
            'velocity': self.velocity
        }

    def get_visual_properties(self):
        """Return properties needed for visualization"""
        return {
            'position': self.position,
            'radius': self.radius,
            'color': self.color,
            'panel_color': self.panel_color
        }