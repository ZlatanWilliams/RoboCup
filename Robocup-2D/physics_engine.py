import numpy as np

class PhysicsEngine:
    def __init__(self):
        self.gravity = 9.81  # m/s²
        self.air_density = 1.225  # kg/m³
        self.ground_friction = 0.96
        self.collision_elasticity = 0.7
        
    def apply_physics(self, entity, dt):
        """Apply physics to any entity with position and velocity"""
        if hasattr(entity, 'velocity') and hasattr(entity, 'position'):
            if entity.velocity is None:
                entity.velocity = np.array([0.0, 0.0])
                
            # Apply velocity
            if np.any(entity.velocity):
                # Calculate air resistance
                speed = np.linalg.norm(entity.velocity)
                if speed > 0:
                    drag_force = -0.5 * self.air_density * speed * speed * 0.47  # 0.47 is drag coefficient
                    drag_acceleration = drag_force / entity.mass
                    entity.velocity += (entity.velocity / speed) * drag_acceleration * dt
                
                # Apply ground friction
                entity.velocity *= self.ground_friction ** dt
                
                # Update position
                entity.position += entity.velocity * dt
                
                # Stop if moving very slowly
                if np.linalg.norm(entity.velocity) < 0.01:
                    entity.velocity = np.array([0.0, 0.0])
    
    def check_collision(self, entity1, entity2):
        """Check if two entities are colliding"""
        if not (hasattr(entity1, 'position') and hasattr(entity2, 'position')):
            return False, 0
            
        distance = np.linalg.norm(entity1.position - entity2.position)
        collision = distance < (entity1.radius + entity2.radius)
        return collision, distance
    
    def resolve_collision(self, entity1, entity2):
        """Handle collision between two entities with improved separation"""
        # Calculate collision normal
        normal = entity2.position - entity1.position
        distance = np.linalg.norm(normal)
        
        if distance == 0:
            # If entities are at exactly the same position, move one slightly
            normal = np.array([1.0, 0.0])
            distance = 0.01
        
        normal = normal / distance
        
        # Calculate relative velocity
        relative_velocity = entity2.velocity - entity1.velocity
        
        # Calculate separation needed
        min_distance = entity1.radius + entity2.radius
        overlap = min_distance - distance
        
        if overlap > 0:
            # Separate the entities
            separation = normal * overlap
            
            # Move both entities apart equally
            entity1.position -= separation * 0.5
            entity2.position += separation * 0.5
            
            # Add a small bounce effect
            bounce_speed = 0.3  # Small bounce speed
            entity1.velocity -= normal * bounce_speed
            entity2.velocity += normal * bounce_speed
            
            # Add a small random component to prevent sticking
            random_dir = np.random.normal(0, 0.1, 2)
            entity1.velocity += random_dir
            entity2.velocity -= random_dir
    
    def apply_kick(self, ball, direction, power):
        """Apply a kick force to the ball"""
        if not np.any(direction):
            return
            
        # Normalize direction
        direction = direction / np.linalg.norm(direction)
        
        # Calculate kick velocity
        kick_speed = power * ball.kick_power_factor
        ball.velocity = direction * kick_speed
        
        # Debug info
        print(f"Kick applied: direction={direction}, power={power}, resulting velocity={ball.velocity}")
        
    def apply_pass(self, ball, start_pos, target_pos, power):
        """Calculate and apply passing velocity"""
        direction = target_pos - start_pos
        distance = np.linalg.norm(direction)
        
        if distance > 0:
            direction = direction / distance
            
            # Adjust power based on distance
            adjusted_power = min(power * (1 + distance/10), power * 1.5)
            
            # Apply kick with adjusted power
            self.apply_kick(ball, direction, adjusted_power)
            print(f"Pass applied: distance={distance}, adjusted_power={adjusted_power}")