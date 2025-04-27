import numpy as np
from fsm import PlayerFSM, PlayerState
from game_engine import GameState

class Player:
    def __init__(self, team, role, x, y, color):
        # Team and role properties
        self.team = team
        self.role = role
        self.color = color
        
        # Physics properties
        self.position = np.array([x, y], dtype=float)
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.acceleration = np.array([0.0, 0.0], dtype=float)
        self.mass = 5.0  # kg
        self.speed = 2.0  # Base movement speed
        self.max_speed = 3.0
        self.radius = 0.15  # Collision radius
        self.size = 0.15  # For rendering
        
        # Game state properties
        self.target_position = self.position.copy()
        self.has_ball = False
        self.ball = None
        self.teammates = []
        self.opponents = []
        
        # Initialize FSM immediately
        self.fsm = PlayerFSM(self)
        
        # Goal positions
        self.opponent_goal = np.array([4.5, 0], dtype=float) if team.team_side == 'home' else np.array([-4.5, 0], dtype=float)
        self.team_goal = np.array([-4.5, 0], dtype=float) if team.team_side == 'home' else np.array([4.5, 0], dtype=float)
        
        # Stealing mechanics
        self.steal_cooldown = 2.0  # Cooldown time in seconds
        self.steal_timer = 0.0  # Current cooldown timer
        self.steal_range = 0.3  # Range for stealing attempt
        self.steal_success_chance = 0.6  # Base 60% chance to steal
        
        # Role-based steal success modifications
        if role == 'striker' or role == 'striker1' or role == 'striker2':
            self.steal_success_chance *= 1.2  # Strikers are better at stealing
        elif role == 'defender' or role == 'defender1' or role == 'defender2':
            self.steal_success_chance *= 1.1  # Defenders are slightly better at stealing
        elif role == 'goalkeeper':
            self.steal_success_chance *= 0.8  # Goalkeepers are worse at stealing

    def update(self, dt):
        # Update stealing cooldown
        if self.steal_timer > 0:
            self.steal_timer = max(0, self.steal_timer - dt)
        
        # Update FSM
        if self.fsm:
            self.fsm.update(dt)
        
        # Update physics
        self.update_physics(dt)
        
        # Clamp velocity to max speed
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = (self.velocity / speed) * self.max_speed

    def update_physics(self, dt):
        # Apply acceleration
        self.velocity += self.acceleration * dt
            
        # Update position
        new_position = self.position + self.velocity * dt
        
        # Handle collisions at new position
        collision_free_pos = self.handle_collisions(new_position)
        
        # Update to collision-free position
        self.position = collision_free_pos
        
        # Reset acceleration
        self.acceleration = np.array([0.0, 0.0])

    def handle_collisions(self, new_position):
        """Handle physical collisions with other players"""
        final_position = new_position.copy()
        
        all_players = self.teammates + self.opponents
        for other in all_players:
            if other == self:
                continue
                
            # Calculate distance at new position
            diff = final_position - other.position
            distance = np.linalg.norm(diff)
            
            # Only handle actual collisions (when players touch)
            min_distance = self.radius + other.radius
            if distance < min_distance:
                # Move back just enough to not overlap
                if distance > 0:
                    direction = diff / distance
                    overlap = min_distance - distance
                    separation = direction * overlap
                    final_position = other.position + direction * min_distance
                    
                    # Reduce velocity in collision direction
                    dot_product = np.dot(self.velocity, direction)
                    if dot_product < 0:  # Only if moving towards other player
                        self.velocity -= direction * dot_product
                
        return final_position

    def move_to(self, target, dt):
        """Move towards target position"""
        direction = target - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.01:
            # Calculate desired velocity
            direction = direction / distance
            desired_speed = min(self.speed, distance)
            desired_velocity = direction * desired_speed
            
            # Calculate steering force
            steering = desired_velocity - self.velocity
            steering = steering * 3.0  # Steering factor
            
            # Apply force
            self.apply_force(steering)

    def apply_force(self, force):
        """Apply a force vector to the player"""
        self.acceleration += force / self.mass

    def can_steal(self):
        """Check if player can attempt to steal"""
        return self.steal_timer <= 0

    def attempt_steal(self, opponent):
        """Attempt to steal the ball from an opponent"""
        if not self.can_steal():
            return False
            
        # Start cooldown
        self.steal_timer = self.steal_cooldown
        
        # Calculate base steal success chance based on various factors
        chance = self.steal_success_chance
        
        # Modify chance based on relative player positions (stealing from behind is easier)
        opponent_to_goal_dir = opponent.opponent_goal - opponent.position
        opponent_to_goal_dir = opponent_to_goal_dir / np.linalg.norm(opponent_to_goal_dir)
        
        self_to_opponent = opponent.position - self.position
        self_to_opponent = self_to_opponent / np.linalg.norm(self_to_opponent)
        
        # Dot product tells us if we're behind the opponent (negative value)
        # or in front of them (positive value)
        angle_factor = np.dot(opponent_to_goal_dir, self_to_opponent)
        
        # Stealing from behind/side is easier than from the front
        if angle_factor < 0:  # We're behind them
            chance *= 1.3
        elif abs(angle_factor) < 0.3:  # We're to their side
            chance *= 1.1
        else:  # We're in front of them
            chance *= 0.8
        
        # Modify chance based on relative speed - more realistic formula
        rel_speed = np.linalg.norm(self.velocity - opponent.velocity)
        if rel_speed > 1.0:
            # If closing in at high speed, harder to make a clean steal
            chance *= max(0.6, 1.0 - (rel_speed - 1.0)/5.0)
        
        # Distance factor is more significant now
        distance = np.linalg.norm(self.position - opponent.position)
        distance_factor = max(0.3, 1.0 - distance/(self.steal_range * 0.8))
        chance *= distance_factor
        
        # Add small random factor for naturalistic variability
        chance *= np.random.uniform(0.9, 1.1)
        
        # Cap the final chance for balance
        chance = min(0.85, max(0.1, chance))
        
        # Print debug info
        print(f"Steal attempt by {self.role} from {opponent.role}, chance: {chance:.2f}")
        
        # Random roll for success
        if np.random.random() < chance:
            print(f"{self.role} successfully stole the ball!")
            return True
        else:
            print(f"{self.role} failed to steal the ball!")
            return False

    def find_best_pass(self):
        """Find best teammate to pass to"""
        best_teammate = None
        best_score = float('-inf')
        
        for teammate in self.teammates:
            if teammate.role in ['striker', 'striker1', 'striker2', 'midfielder']:
                dist_to_goal = np.linalg.norm(teammate.position - self.opponent_goal)
                dist_to_teammate = np.linalg.norm(self.position - teammate.position)
                
                if dist_to_teammate < 4.0:  # Maximum pass distance
                    score = -dist_to_goal - (dist_to_teammate * 0.5)
                    
                    if score > best_score:
                        best_score = score
                        best_teammate = teammate
        
        return best_teammate

    def is_in_shooting_position(self):
        """Check if player is in a good position to shoot"""
        dist_to_goal = np.linalg.norm(self.position - self.opponent_goal)
        
        # Check if player is on the correct side of the field
        in_attacking_half = (self.position[0] > 0 if self.team.team_side == 'home' 
                           else self.position[0] < 0)
        
        # Check if within shooting range and in attacking half
        is_close = dist_to_goal < 2.5  # Increased shooting range slightly
        
        return is_close and in_attacking_half

    def get_visual_properties(self):
        """Get properties for rendering"""
        team_identifier = 'H' if self.team.team_side == 'home' else 'A'
        return {
            'position': self.position,
            'color': self.color,
            'size': self.size,
            'label': f"{team_identifier}-{self.role[0].upper()}"
        }
    
    def get_all_strikers(self):
        """Get all striker players in the team"""
        strikers = []
        for player in self.players:
            if player.role.startswith('striker'):
                strikers.append(player)
        return strikers
    
    def are_multiple_strikers(self):
        """Check if team has multiple strikers"""
        striker_count = 0
        for player in self.players:
            if player.role.startswith('striker'):
                striker_count += 1
        return striker_count > 1
    
    def take_kickoff(self):
        """
        Special method for a player to take a kickoff.
        This should only be called by the striker of the kickoff team.
        """
        if not self.has_ball and self.ball:
            # Go to the ball first if we don't have it
            direction = self.ball.position - self.position
            distance = np.linalg.norm(direction)
            
            if distance < self.radius + self.ball.radius:
                # We're close enough to the ball to take kickoff
                self.has_ball = True
                self.ball.owner = self
                print(f"{self.role} taking kickoff!")
                
                # Pass forward to teammate
                best_teammate = None
                best_forward_pos = float('-inf')
                
                for teammate in self.teammates:
                    # Find the most forward teammate
                    forward_pos = 0
                    if self.team.team_side == 'home':
                        forward_pos = teammate.position[0]  # Higher X is more forward for home team
                    else:
                        forward_pos = -teammate.position[0]  # Lower X is more forward for away team
                    
                    if forward_pos > best_forward_pos:
                        best_forward_pos = forward_pos
                        best_teammate = teammate
                
                if best_teammate:
                    # Pass to the forward teammate
                    direction = best_teammate.position - self.position
                    dist = np.linalg.norm(direction)
                    
                    if dist > 0:
                        # Prepare kickoff pass
                        direction = direction / dist
                        
                        # Adjust power for medium-range pass
                        power = 0.6
                        if dist > 3.0:
                            power = 0.7
                        
                        # Apply pass
                        self.ball.pass_to(best_teammate.position, power)
                        print(f"Kickoff pass to {best_teammate.role}!")
                        
                        # Update player state
                        self.has_ball = False
                        self.fsm.change_state(PlayerState.POSITIONING)
                        return True
                
                # If no teammate found, kick forward
                if self.team.team_side == 'home':
                    direction = np.array([1.0, 0.0])  # Forward for home team
                else:
                    direction = np.array([-1.0, 0.0])  # Forward for away team
                
                # Add slight randomization
                direction[1] = np.random.uniform(-0.2, 0.2)
                if np.linalg.norm(direction) > 0:
                    direction = direction / np.linalg.norm(direction)
                
                # Execute kickoff
                self.ball.kick(direction, 0.7)
                print(f"{self.role} kicks off forward!")
                
                # Update player state
                self.has_ball = False
                self.fsm.change_state(PlayerState.POSITIONING)
                return True
                
            return False

    # Add this to PursueBallState in fsm.py

    def execute(self, dt):
        if not self.player.ball:
            self.player.fsm.change_state(PlayerState.POSITIONING)
            return
                
        # Special kickoff handling
        if self._is_kickoff_state() and self._is_kickoff_player():
            # If close to ball, take kickoff
            ball_dist = np.linalg.norm(self.player.position - self.player.ball.position)
            if ball_dist < self.player.radius + self.player.ball.radius + 0.1:
                if hasattr(self.player, 'take_kickoff'):
                    self.player.take_kickoff()
                    return
        
        # Regular behavior (rest of original PursueBallState code)
        # ...

    def _is_kickoff_state(self):
        """Check if we're in kickoff state"""
        if hasattr(self.player, 'team') and hasattr(self.player.team, 'game_engine'):
            return self.player.team.game_engine.game_state == GameState.KICKOFF
        return False

    def _is_kickoff_player(self):
        """Check if this player should take the kickoff"""
        if not hasattr(self.player, 'team') or not hasattr(self.player.team, 'game_engine'):
            return False
            
        game_engine = self.player.team.game_engine
        is_kickoff_team = self.player.team.team_side == game_engine.kickoff_team
        
        # Only striker on kickoff team should take kickoff
        return is_kickoff_team and (self.player.role == 'striker' or self.player.role.startswith('striker'))
class Team:
    def __init__(self, team_side, strategy_type='balanced'):
        self.team_side = team_side  # 'home' or 'away'
        self.color = (255, 50, 50) if team_side == 'home' else (50, 50, 255)
        self.strategy_type = strategy_type
        
        # Initialize strategy based on type
        if strategy_type == 'balanced':
            from behaviors import BalancedStrategy
            self.strategy = BalancedStrategy(team_side)
        elif strategy_type == 'defensive':
            from additional_strategies import DefensiveStrategy
            self.strategy = DefensiveStrategy(team_side)
        elif strategy_type == 'offensive':
            from additional_strategies import OffensiveStrategy
            self.strategy = OffensiveStrategy(team_side)
        else:
            from behaviors import MinimalStrategy
            self.strategy = MinimalStrategy(team_side)
            
        self.players = self.initialize_players()
        
        # Set team reference for all players
        for player in self.players:
            player.team = self
            
        # Set teammates for each player
        for player in self.players:
            player.teammates = [p for p in self.players if p != player]

    def update(self, dt, ball_pos, tactical_info=None):
        # Update strategy with tactical information if available
        if tactical_info:
            game_phase = tactical_info['game_phase']
            optimal_positions = self.strategy.tactical_engine.calculate_team_positions(
                self, ball_pos, game_phase
            )
            
            # Update target positions based on tactical analysis
            for player in self.players:
                if player.role in optimal_positions:
                    player.target_position = optimal_positions[player.role]
        
        # Regular strategy update
        self.strategy.update_player_positions(self.players, ball_pos)
        
        # Update individual players
        for player in self.players:
            player.update(dt)

    def initialize_players(self):
        if self.strategy_type == 'defensive':
            if self.team_side == 'home':
                return [
                    Player(self, 'goalkeeper', -4.3, 0, self.color),
                    Player(self, 'defender1', -3.0, -1.0, self.color),
                    Player(self, 'defender2', -3.0, 1.0, self.color),
                    Player(self, 'striker', -0.8, 0.0, self.color)
                ]
            else:
                return [
                    Player(self, 'goalkeeper', 4.3, 0, self.color),
                    Player(self, 'defender1', 3.0, -1.0, self.color),
                    Player(self, 'defender2', 3.0, 1.0, self.color),
                    Player(self, 'striker', 0.8, 0.0, self.color)
                ]
        elif self.strategy_type == 'offensive':
            if self.team_side == 'home':
                return [
                    Player(self, 'goalkeeper', -4.3, 0, self.color),
                    Player(self, 'defender', -3.0, 0, self.color),
                    Player(self, 'striker1', -0.8, -1.0, self.color),
                    Player(self, 'striker2', -0.8, 1.0, self.color)
                ]
            else:
                return [
                    Player(self, 'goalkeeper', 4.3, 0, self.color),
                    Player(self, 'defender', 3.0, 0, self.color),
                    Player(self, 'striker1', 0.8, -1.0, self.color),
                    Player(self, 'striker2', 0.8, 1.0, self.color)
                ]
        else:  # balanced strategy (default)
            if self.team_side == 'home':
                return [
                    Player(self, 'goalkeeper', -4.3, 0, self.color),
                    Player(self, 'defender', -3.0, 0, self.color),
                    Player(self, 'midfielder', -1.5, 1.0, self.color),
                    Player(self, 'striker', -0.8, -1.0, self.color)
                ]
            else:
                return [
                    Player(self, 'goalkeeper', 4.3, 0, self.color),
                    Player(self, 'defender', 3.0, 0, self.color),
                    Player(self, 'midfielder', 1.5, -1.0, self.color),
                    Player(self, 'striker', 0.8, 1.0, self.color)
                ]
                
    def get_player_by_role(self, role):
        """Get a player by their role"""
        for player in self.players:
            if player.role == role:
                return player
        return None

    def get_players_in_zone(self, zone):
        """Get all players in a specific zone"""
        players_in_zone = []
        for player in self.players:
            if zone['min_x'] <= player.position[0] <= zone['max_x']:
                players_in_zone.append(player)
        return players_in_zone