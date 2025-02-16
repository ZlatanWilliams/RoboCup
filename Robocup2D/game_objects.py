import math

class Ball:
    def __init__(self):
        # FIFA Size 1 specifications
        self.circumference = 0.45  # 45cm
        self.diameter = 0.143       # 14.3cm
        self.radius = self.diameter / 2
        self.mass = 0.31           # kg (typical for size 1)
        self.leather_thickness = 0.002  # 2mm
        
        # Physics properties
        self.position = (0, 0)
        self.velocity = (0, 0)
        self.decay = 0.96          # Leather-air friction
        self.bounce_coefficient = 0.65  # Leather surface bounce
        
        # Visual properties
        self.color = '#8B4513'     # Saddle brown (leather)
        self.panel_color = '#A0522D'  # Darker brown for panels
        self.panels = 12           # Typical panel count

    def update(self, dt):
        """Realistic physics update with air resistance"""
        # Apply velocity decay with quadratic air resistance
        speed = math.hypot(*self.velocity)
        if speed > 0:
            drag_factor = 0.5 * 1.2 * 0.38 * (speed**2)  # ρ=1.2, C_d=0.38
            self.velocity = (
                self.velocity[0] - (drag_factor * self.velocity[0]/speed) * dt,
                self.velocity[1] - (drag_factor * self.velocity[1]/speed) * dt
            )
        
        # Apply basic decay
        self.velocity = (
            self.velocity[0] * self.decay,
            self.velocity[1] * self.decay
        )
        
        # Update position
        self.position = (
            self.position[0] + self.velocity[0] * dt,
            self.position[1] + self.velocity[1] * dt
        )
        
class Goal:
    def __init__(self, side):
        self.side = side  # 'left' or 'right'
        self.score = 0