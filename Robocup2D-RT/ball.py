import numpy as np

class Ball:
    def __init__(self):
        # Ball specifications
        self.circumference = 0.45  # meters
        self.diameter = 0.143
        self.radius = self.diameter / 2
        self.mass = 0.31  # kg
        self.last_owner = None
        
        # Physics properties
        self.position = np.array([0.0, 0.0], dtype=float)
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.friction = 0.98  # Slightly increased friction
        self.bounce_coefficient = 0.65  # Reduced bounce
        self.kick_power_factor = 5.0  # Reduced kick power
        self.max_speed = 8.0  # Reduced max speed
        
        # Game state
        self.owner = None
        
        # Add these new properties:
        self.last_owner = None
        self.pass_cooldown = 0.0  # Cooldown time in seconds

    def update(self, dt):
        # Update last_owner when ball is released
        if self.owner is None and hasattr(self, 'last_owner') and self.last_owner is not None:
            # Keep last_owner for a short time after release
            if hasattr(self, 'pass_cooldown') and self.pass_cooldown > 0:
                self.pass_cooldown -= dt
                if self.pass_cooldown <= 0:
                    # Clear last_owner after cooldown expires
                    self.last_owner = None
                    self.pass_cooldown = 0
        
        if self.owner:
            # Calculate offset based on team side
            if self.owner.team.team_side == 'home':
                offset = np.array([0.3, 0])  # Ball in front for home team
            else:
                offset = np.array([-0.3, 0])  # Ball in front for away team
            
            self.position = self.owner.position + offset
            self.velocity = np.array([0.0, 0.0])
            return
            
        # Apply physics when ball is free
        # Apply friction to velocity
        self.velocity *= self.friction
        
        # Apply velocity to position
        self.position += self.velocity * dt
        
        # Cap speed
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = (self.velocity / speed) * self.max_speed
        
        # Stop ball if moving very slowly
        if speed < 0.1:
            self.velocity = np.array([0.0, 0.0])

    # Enhanced kick method with variable speed and trajectory
    def kick(self, direction, power=0.7):
        """Kick the ball in a direction with realistic physics"""
        if self.owner is not None:
            if np.any(direction):
                direction = direction / np.linalg.norm(direction)
                
                # Calculate kick speed with slightly randomized power
                actual_power = power * np.random.uniform(0.9, 1.1)
                kick_speed = actual_power * self.kick_power_factor
                
                # Add slight randomization to the direction (imperfect kicks)
                random_factor = (1.0 - power) * 0.1  # Higher power = more accurate
                random_angle = np.random.normal(0, random_factor)
                
                # Rotate direction slightly
                cos_theta = np.cos(random_angle)
                sin_theta = np.sin(random_angle)
                new_x = direction[0] * cos_theta - direction[1] * sin_theta
                new_y = direction[0] * sin_theta + direction[1] * cos_theta
                direction = np.array([new_x, new_y])
                direction = direction / np.linalg.norm(direction)
                
                # Store a reference to previous owner
                self.last_owner = self.owner
                
                # CRITICAL: Clear ownership BEFORE setting velocity
                self.owner.has_ball = False
                self.owner = None
                
                # Set velocity
                self.velocity = direction * kick_speed
                
                print(f"Ball kicked with velocity: {self.velocity}, speed: {np.linalg.norm(self.velocity):.3f}")
        elif self.velocity is not None:
            # This allows shooting even when the ball isn't owned but is moving
            # (Important for deflections and quick shots)
            print("Kicking a ball that isn't owned")
            if np.any(direction):
                direction = direction / np.linalg.norm(direction)
                kick_speed = power * self.kick_power_factor
                self.velocity = direction * kick_speed

    def pass_to(self, target_pos, power=0.7):
        """Pass the ball to a target position with proper ownership release"""
        print(f"PASS_TO CALLED: Current owner: {self.owner.role if self.owner else 'None'}")
        print(f"PASS_TO target: {target_pos}, power: {power}")
        
        if self.owner is not None:
            direction = target_pos - self.position
            distance = np.linalg.norm(direction)
            
            if distance > 0:
                direction = direction / distance
                
                # Adjust power based on distance
                if distance < 1.0:
                    adjusted_power = power * 0.8  # Gentler for short passes
                elif distance < 3.0:
                    adjusted_power = power * (0.8 + 0.2 * (distance - 1.0) / 2.0)
                else:
                    adjusted_power = power * (1.0 + (distance - 3.0) * 0.07)
                    
                adjusted_power = min(adjusted_power, power * 1.4)
                
                # Store a reference to previous owner
                previous_owner = self.owner
                self.last_owner = previous_owner
                
                # CRITICAL: Clear ownership before changing velocity
                previous_owner.has_ball = False
                self.owner = None
                
                # Set pass cooldown (0.3 seconds is enough)
                self.pass_cooldown = 0.3
                
                # Now apply kick with adjusted power
                self.velocity = direction * adjusted_power * self.kick_power_factor
                
                print(f"PASS EXECUTED: From {previous_owner.role} with power: {adjusted_power:.3f}")
                print(f"  Target: {target_pos}, Direction: {direction}, Velocity: {self.velocity}")
            else:
                print("ERROR: Pass distance is zero!")
        else:
            print("ERROR: Trying to pass without an owner!")