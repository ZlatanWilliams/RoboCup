import numpy as np
from enum import Enum

class PlayerState(Enum):
    POSITIONING = "positioning"
    PURSUE_BALL = "pursue_ball"
    POSSESSION = "possession"

class FSMState:
    def __init__(self, player):
        self.player = player

    def enter(self):
        pass

    def execute(self, dt):
        pass

    def exit(self):
        pass

def seek(player, target_pos, dt):
    """Move towards a target position"""
    direction = target_pos - player.position
    distance = np.linalg.norm(direction)
    
    if distance > 0.01:
        player.move_to(target_pos, dt)

class PositioningState(FSMState):
    def execute(self, dt):
        # Move to tactical position if we're far from it
        if hasattr(self.player, 'target_position'):
            distance_to_target = np.linalg.norm(self.player.position - self.player.target_position)
            if distance_to_target > 0.1:
                seek(self.player, self.player.target_position, dt)
        
        # Check if we should go for the ball
        if self.player.ball is not None:
            ball_pos = self.player.ball.position
            zone = self.player.team.strategy.zones[self.player.role]
            my_pos = self.player.position
            
            # ENHANCED: More aggressive ball pursuit for strikers
            if self.player.role.startswith('striker'):
                # Strikers use expanded zone checking and longer pursuit range
                ball_dist = np.linalg.norm(my_pos - ball_pos)
                
                # Expanded pursuit range for strikers
                can_reach = ball_dist < 3.0  # Longer pursuit range for strikers (was 2.0)
                
                # Expanded zone check for strikers - use opponent half or extended range
                is_in_opponent_half = (ball_pos[0] > 0 if self.player.team.team_side == 'home' 
                                     else ball_pos[0] < 0)
                
                # Strikers go for ball if it's:
                # 1. In opponent half, OR
                # 2. Within extended pursuit range, OR
                # 3. In their normal zone
                if (is_in_opponent_half or can_reach) and not self.player.ball.owner:
                    print(f"Striker {self.player.role} aggressively pursuing ball!")
                    self.player.fsm.change_state(PlayerState.PURSUE_BALL)
                    return
            
            # ENHANCED: More aggressive ball pursuit for defenders
            elif 'defender' in self.player.role:
                ball_dist = np.linalg.norm(my_pos - ball_pos)
                
                # Expanded pursuit range for defenders - not as far as strikers but more than default
                can_reach = ball_dist < 2.5  # Increased pursuit range (was 2.0)
                
                # Check if ball is in our half
                is_in_our_half = (ball_pos[0] < 0 if self.player.team.team_side == 'home' 
                                else ball_pos[0] > 0)
                
                # Defenders aggressively pursue ball in our half or within extended range
                if (is_in_our_half and ball_dist < 3.0) or can_reach:
                    if not self.player.ball.owner:
                        print(f"Defender {self.player.role} aggressively pursuing ball!")
                        self.player.fsm.change_state(PlayerState.PURSUE_BALL)
                        return
            
            # Normal ball pursuit for other roles
            ball_in_zone = (zone["min_x"] <= ball_pos[0] <= zone["max_x"] and 
                          zone["min_y"] <= ball_pos[1] <= zone["max_y"])
            
            ball_dist = np.linalg.norm(my_pos - ball_pos)
            can_reach = ball_dist < 2.0  # Standard pursuit range
            
            if ball_in_zone and can_reach and not self.player.ball.owner:
                self.player.fsm.change_state(PlayerState.PURSUE_BALL)

class PursueBallState(FSMState):
    def execute(self, dt):
        if not self.player.ball:
            self.player.fsm.change_state(PlayerState.POSITIONING)
            return
            
        if self.player.ball.owner and self.player.ball.owner != self.player:
            # Aggressive ball stealing for strikers
            if self.player.role.startswith('striker'):
                ball_owner = self.player.ball.owner
                
                # Only try to steal if ball is owned by opponent team
                if ball_owner.team != self.player.team:
                    distance_to_ball_owner = np.linalg.norm(self.player.position - ball_owner.position)
                    
                    # If striker is close enough to ball owner, attempt steal
                    if distance_to_ball_owner < 0.8:  # Close stealing range
                        if self.player.can_steal():
                            print(f"Striker {self.player.role} attempting aggressive steal")
                            # Continue pursuit to facilitate stealing
                            seek(self.player, self.player.ball.position, dt)
                            return
            
            # ENHANCED: Defenders also attempt steals in our half
            elif 'defender' in self.player.role:
                ball_owner = self.player.ball.owner
                ball_pos = self.player.ball.position
                
                # Check if ball is in our half
                is_in_our_half = (ball_pos[0] < 0 if self.player.team.team_side == 'home' 
                                else ball_pos[0] > 0)
                
                # Only try to steal if ball is owned by opponent team and in our half
                if ball_owner.team != self.player.team and is_in_our_half:
                    distance_to_ball_owner = np.linalg.norm(self.player.position - ball_owner.position)
                    
                    # If defender is close enough to ball owner, attempt steal
                    if distance_to_ball_owner < 1.0:  # Slightly larger range than strikers
                        if self.player.can_steal():
                            print(f"Defender {self.player.role} attempting defensive steal")
                            # Continue pursuit to facilitate stealing
                            seek(self.player, self.player.ball.position, dt * 1.05)  # Small speed boost
                            return
            
            # Otherwise, go back to positioning
            self.player.fsm.change_state(PlayerState.POSITIONING)
            return
        
        # Actively chase the ball
        ball_pos = self.player.ball.position
        zone = self.player.team.strategy.zones[self.player.role]
        
        # ENHANCED: Strikers chase ball more aggressively
        if self.player.role.startswith('striker'):
            # Strikers always chase ball if not owned
            if not self.player.ball.owner:
                print(f"Striker {self.player.role} chasing loose ball")
                seek(self.player, ball_pos, dt * 1.1)  # 10% speed boost for strikers
            else:
                self.player.fsm.change_state(PlayerState.POSITIONING)
        
        # ENHANCED: Defenders also chase aggressively in certain situations
        elif 'defender' in self.player.role:
            # Check if ball is in our half
            is_in_our_half = (ball_pos[0] < 0 if self.player.team.team_side == 'home' 
                            else ball_pos[0] > 0)
            
            ball_in_zone = (zone["min_x"] <= ball_pos[0] <= zone["max_x"] and 
                         zone["min_y"] <= ball_pos[1] <= zone["max_y"])
            
            # Defenders chase more aggressively in our half
            if is_in_our_half or ball_in_zone:
                if not self.player.ball.owner:
                    print(f"Defender {self.player.role} pursuing loose ball")
                    # Small speed boost when defending our half
                    boost = 1.05 if is_in_our_half else 1.0
                    seek(self.player, ball_pos, dt * boost)
                    return
            
            self.player.fsm.change_state(PlayerState.POSITIONING)
        else:
            # Other players only chase within their zone or midfielder
            ball_in_zone = (zone["min_x"] <= ball_pos[0] <= zone["max_x"] and 
                       zone["min_y"] <= ball_pos[1] <= zone["max_y"])
            
            if ball_in_zone or self.player.role in ['midfielder', 'striker']:
                # Move directly to ball
                seek(self.player, ball_pos, dt)
            else:
                self.player.fsm.change_state(PlayerState.POSITIONING)

class PossessionState(FSMState):
    def __init__(self, player):
        super().__init__(player)
        self.hold_time = 0
        # Direction properties
        self.dribble_direction = None
        self.direction_change_timer = 0
        
    def enter(self):
        self.hold_time = 0
        self.direction_change_timer = 0
        
        # Initialize direction based on role
        if self.player.team.team_side == 'home':
            base_dir = np.array([1.0, 0.0])  # Forward for home team
        else:
            base_dir = np.array([-1.0, 0.0])  # Forward for away team
            
        # Add small random component
        random_y = np.random.uniform(-0.2, 0.2)
        self.dribble_direction = base_dir + np.array([0, random_y])
        
        # Normalize
        if np.linalg.norm(self.dribble_direction) > 0:
            self.dribble_direction = self.dribble_direction / np.linalg.norm(self.dribble_direction)
            
        print(f"DEBUG: {self.player.role} entered POSSESSION with direction {self.dribble_direction}")

    def execute(self, dt):
        if not self.player.has_ball or not self.player.ball:
            self.player.fsm.change_state(PlayerState.POSITIONING)
            return

        self.hold_time += dt
        self.direction_change_timer += dt
        
        # Role-specific behaviors
        if self.player.role == "goalkeeper":
            self.handle_goalkeeper_behavior(dt)
        elif self.player.role.startswith("striker"):
            self.handle_striker_behavior(dt)
        elif self.player.role == "midfielder":
            self.handle_midfielder_behavior(dt)
        else:  # defender
            self.handle_defender_behavior(dt)

    def handle_goalkeeper_behavior(self, dt):
        """Enhanced goalkeeper behavior to avoid passing loops"""
        # Track who passed to the goalkeeper
        if not hasattr(self, 'received_pass_from'):
            self.received_pass_from = None

        # Check where the ball came from
        if self.hold_time < 0.1 and self.player.ball.last_owner:
            self.received_pass_from = self.player.ball.last_owner
            print(f"Goalkeeper received pass from {self.received_pass_from.role}")

        # Don't hold the ball too long
        if self.hold_time > 0.8:
            # 1. First option: Find an open midfielder or striker
            midfielder = self.find_teammate_by_role('midfielder')
            striker = self.find_teammate_by_role_prefix('striker')
            
            viable_targets = []
            
            # Check if midfielder is viable
            if midfielder and self.is_good_pass_target(midfielder):
                # Higher weight if didn't receive from them
                weight = 3.0 if self.received_pass_from != midfielder else 1.0
                viable_targets.append((midfielder, weight))
            
            # Check if striker is viable for long passes
            if striker and self.is_good_pass_target(striker):
                # Higher weight if didn't receive from them
                weight = 2.0 if self.received_pass_from != striker else 0.5
                viable_targets.append((striker, weight))
            
            # 2. Find defenders but avoid passing back to who passed to us
            for teammate in self.player.teammates:
                if 'defender' in teammate.role:
                    # Severely penalize passing back to same defender
                    if teammate == self.received_pass_from:
                        # Only pass back if absolutely necessary
                        if self.is_good_pass_target(teammate):
                            viable_targets.append((teammate, 0.2))  # Very low weight
                    else:
                        # Prefer other defenders
                        if self.is_good_pass_target(teammate):
                            viable_targets.append((teammate, 1.5))
            
            # Choose the best option based on weighted random selection
            if viable_targets:
                # Sort by weight for more predictable behavior
                viable_targets.sort(key=lambda x: x[1], reverse=True)
                
                # Use weights for randomized but biased selection
                weights = [target[1] for target in viable_targets]
                total_weight = sum(weights)
                if total_weight > 0:
                    # Normalize weights
                    weights = [w/total_weight for w in weights]
                    
                    # Make selection based on weights
                    import random
                    r = random.random()
                    cumulative = 0
                    for i, w in enumerate(weights):
                        cumulative += w
                        if r <= cumulative:
                            selected_target = viable_targets[i][0]
                            print(f"Goalkeeper passing to {selected_target.role} (weighted choice)")
                            self.execute_pass_to(selected_target)
                            return
                
                # Fallback to first target if weights don't work
                first_target = viable_targets[0][0]
                print(f"Goalkeeper passing to {first_target.role} (fallback)")
                self.execute_pass_to(first_target)
                return
            
            # 3. If no viable targets, clear the ball upfield
            print("No good passing options, goalkeeper clearing ball")
            self.clear_ball()

    def handle_striker_behavior(self, dt):
        """Striker behavior - focuses on moving toward goal and shooting"""
        dist_to_goal = np.linalg.norm(self.player.position - self.player.opponent_goal)
        
        # First priority - shoot if close to goal
        if dist_to_goal < 2.5:  # Increased shooting range for strikers
            print(f"Striker in shooting position! Distance: {dist_to_goal:.2f}")
            self.shoot_at_goal()
            return
            
        # Second priority - check for other striker to pass to
        other_striker = self.find_other_striker()
        
        # Only pass if in good position and held ball long enough
        if other_striker and self.hold_time > 1.5:  # Reduced hold time for quicker passing
            # Check if other striker is in a better scoring position
            other_dist_to_goal = np.linalg.norm(other_striker.position - self.player.opponent_goal)
            
            if (other_dist_to_goal < dist_to_goal or self.under_pressure()) and self.is_good_pass_target(other_striker):
                print(f"Striker passing to another striker")
                self.execute_pass_to(other_striker)
                return
        
        # If under heavy pressure, look for any pass
        if self.under_heavy_pressure():
            any_teammate = self.find_open_teammate()
            if any_teammate:
                print(f"Striker under pressure, passing to {any_teammate.role}")
                self.execute_pass_to(any_teammate)
                return
        
        # Change direction occasionally
        if self.direction_change_timer > 1.5:
            self.change_dribble_direction()
            
        # Dribble in current direction with goal bias
        self.dribble_with_direction(dt, goal_bias=0.8)  # Increased goal focus (80%)

    def handle_midfielder_behavior(self, dt):
        """Midfielder behavior - focuses on distribution and forward support"""
        # Check if should pass (more frequent than striker)
        if self.hold_time > 2.0:
            # Look for striker first
            striker = self.find_teammate_by_role_prefix('striker')
            if striker and self.is_good_pass_target(striker):
                print(f"Midfielder passing to striker")
                self.execute_pass_to(striker)
                return
                
            # Otherwise any open teammate
            teammate = self.find_open_teammate()
            if teammate:
                print(f"Midfielder passing to {teammate.role}")
                self.execute_pass_to(teammate)
                return
                
        # Change direction occasionally
        if self.direction_change_timer > 1.2:  # More frequent direction changes than striker
            self.change_dribble_direction()
            
        # Dribble with current direction and less goal focus
        self.dribble_with_direction(dt, goal_bias=0.5)  # 50% goal influence

    def handle_defender_behavior(self, dt):
        """Enhanced defender behavior to avoid passing loops with goalkeeper"""
        # Track who passed to the defender
        if not hasattr(self, 'received_pass_from'):
            self.received_pass_from = None

        # Check where the ball came from
        if self.hold_time < 0.1 and self.player.ball.last_owner:
            self.received_pass_from = self.player.ball.last_owner
            print(f"Defender received pass from {self.received_pass_from.role}")
        
        # Decide whether to pass or dribble based on situation
        is_from_goalkeeper = self.received_pass_from and self.received_pass_from.role == 'goalkeeper'
        ball_pos = self.player.ball.position
        is_in_opponent_half = (ball_pos[0] > 0 if self.player.team.team_side == 'home' 
                            else ball_pos[0] < 0)
        under_pressure = self.under_pressure()
        
        # Special case: if we just received from goalkeeper and aren't under pressure, 
        # prefer to dribble forward first
        if is_from_goalkeeper and not under_pressure and self.hold_time < 2.0:
            # Dribble more aggressively
            goal_bias = 0.5 if is_in_opponent_half else 0.4
            self.dribble_with_direction(dt, goal_bias=goal_bias)
            return
        
        # Normal passing logic with enhanced goalkeeper avoidance
        pass_timer_threshold = 2.0
        if under_pressure:
            pass_timer_threshold = 1.2
        if is_in_opponent_half:
            pass_timer_threshold = 2.5
            
        # Make passing decisions
        if self.hold_time > pass_timer_threshold:
            # List of potential targets with weights
            viable_targets = []
            
            # When in opponent half, look for strikers first
            if is_in_opponent_half:
                striker = self.find_teammate_by_role_prefix('striker')
                if striker and self.is_good_pass_target(striker):
                    weight = 3.0  # High weight for attacking passes
                    viable_targets.append((striker, weight))
            
            # Look for midfielder first (unless we got the ball from them)
            midfielder = self.find_teammate_by_role('midfielder')
            if midfielder and self.is_good_pass_target(midfielder):
                weight = 2.5 if midfielder != self.received_pass_from else 1.0
                viable_targets.append((midfielder, weight))
            
            # Consider other defender but with lower priority if under pressure
            other_defenders = []
            for teammate in self.player.teammates:
                if 'defender' in teammate.role and teammate != self.player:
                    other_defenders.append(teammate)
                    
            for defender in other_defenders:
                if self.is_good_pass_target(defender):
                    # Lower weight when under pressure (prioritize forward passes)
                    weight = 1.0 if not under_pressure else 0.5
                    viable_targets.append((defender, weight))
                    
            # Only pass back to goalkeeper as a last resort and when under pressure
            goalkeeper = self.find_teammate_by_role('goalkeeper')
            if goalkeeper and goalkeeper != self.received_pass_from and under_pressure:
                if self.is_good_pass_target(goalkeeper):
                    # Very low weight - last resort
                    viable_targets.append((goalkeeper, 0.3))
            
            # Find any other open teammates
            for teammate in self.player.teammates:
                # Skip already evaluated players
                if teammate in [t[0] for t in viable_targets]:
                    continue
                
                if self.is_good_pass_target(teammate):
                    # Default weight
                    viable_targets.append((teammate, 1.0))
            
            # Select from viable targets
            if viable_targets:
                # Sort by weight for more predictable behavior
                viable_targets.sort(key=lambda x: x[1], reverse=True)
                
                # Use weights for randomized but biased selection
                import random
                weights = [target[1] for target in viable_targets]
                total_weight = sum(weights)
                if total_weight > 0:
                    # Normalize weights
                    weights = [w/total_weight for w in weights]
                    
                    # Make selection based on weights
                    r = random.random()
                    cumulative = 0
                    for i, w in enumerate(weights):
                        cumulative += w
                        if r <= cumulative:
                            selected_target = viable_targets[i][0]
                            print(f"Defender passing to {selected_target.role} (weighted choice)")
                            self.execute_pass_to(selected_target)
                            return
                
                # Fallback to first target if weights don't work
                first_target = viable_targets[0][0]
                print(f"Defender passing to {first_target.role} (fallback)")
                self.execute_pass_to(first_target)
                return
                
            # If under heavy pressure and no good pass, clear the ball
            if self.under_heavy_pressure():
                self.clear_ball()
                return
    
        # If we get here, continue dribbling
        goal_bias = 0.3  # Standard conservative bias
        if is_in_opponent_half:
            goal_bias = 0.5  # More goal-focused in opponent half
            
        # Dribble with appropriate direction bias
        self.dribble_with_direction(dt, goal_bias=goal_bias)
        
    # ===== HELPER METHODS =====
    
    def under_pressure(self):
        """Check if player is under significant pressure"""
        pressure_count = 0
        for opponent in self.player.opponents:
            dist = np.linalg.norm(opponent.position - self.player.position)
            if dist < 1.2:  # Close opponent
                pressure_count += 1
        return pressure_count > 0
    
    def under_heavy_pressure(self):
        """Check if player is under heavy pressure"""
        pressure_count = 0
        for opponent in self.player.opponents:
            dist = np.linalg.norm(opponent.position - self.player.position)
            if dist < 0.8:  # Very close opponent
                pressure_count += 1
        return pressure_count >= 2  # Two or more very close opponents
        
    def find_other_striker(self):
        """Find another striker on the team"""
        for teammate in self.player.teammates:
            if teammate.role.startswith('striker') and teammate != self.player:
                return teammate
        return None
        
    def find_teammate_by_role(self, role):
        """Find a teammate with exact role"""
        for teammate in self.player.teammates:
            if teammate.role == role:
                return teammate
        return None
        
    def find_teammate_by_role_prefix(self, prefix):
        """Find a teammate whose role starts with prefix"""
        for teammate in self.player.teammates:
            if teammate.role.startswith(prefix):
                return teammate
        return None
        
    def find_open_teammate(self):
        """Find any open teammate for passing"""
        best_teammate = None
        best_score = float('-inf')
        
        for teammate in self.player.teammates:
            # Don't pass to self
            if teammate == self.player:
                continue
                
            pass_dist = np.linalg.norm(teammate.position - self.player.position)
            
            # Skip if too close or too far
            if pass_dist < 0.5 or pass_dist > 4.5:
                continue
                
            # Check how open they are
            is_open = self.is_good_pass_target(teammate)
            if not is_open:
                continue
                
            # Calculate a score based on position and role
            score = 0
            
            # Prefer forward passes
            my_goal_dist = np.linalg.norm(self.player.position - self.player.opponent_goal)
            their_goal_dist = np.linalg.norm(teammate.position - self.player.opponent_goal)
            
            if their_goal_dist < my_goal_dist:
                score += 5  # Bonus for forward passes
                
            # Role bonus
            if teammate.role.startswith('striker'):
                score += 3
            elif teammate.role == 'midfielder':
                score += 2
                
            # Prefer medium distance passes
            dist_score = 5 - abs(pass_dist - 2.5)
            score += dist_score
            
            if score > best_score:
                best_score = score
                best_teammate = teammate
                
        return best_teammate
        
    def is_good_pass_target(self, target):
        """Check if target is a good pass option"""
        # Calculate pass parameters
        pass_vector = target.position - self.player.position
        pass_dist = np.linalg.norm(pass_vector)
        
        if pass_dist < 0.5 or pass_dist > 4.5:
            return False  # Too close or too far
            
        # Check if pass lane is clear
        if pass_dist > 0:
            pass_dir = pass_vector / pass_dist
            
            # Check for opponents in passing lane
            for opponent in self.player.opponents:
                to_opponent = opponent.position - self.player.position
                opponent_dist = np.linalg.norm(to_opponent)
                
                if opponent_dist < pass_dist:
                    # Project opponent onto pass direction
                    proj = np.dot(to_opponent, pass_dir)
                    
                    if 0 < proj < pass_dist:
                        # Calculate perpendicular distance to pass line
                        closest = np.linalg.norm(to_opponent - proj * pass_dir)
                        
                        if closest < 0.4:  # Too close to passing lane
                            return False
        
        return True
        
    def execute_pass_to(self, target):
        """Execute a pass to target with proper ownership handling"""
        if not self.player.has_ball or not self.player.ball:
            return False
            
        try:
            # Calculate pass details
            pass_vector = target.position - self.player.position
            pass_dist = np.linalg.norm(pass_vector)
            
            if pass_dist <= 0:
                return False  # Can't pass to self or same position
                
            pass_dir = pass_vector / pass_dist
            
            # Add randomization to target
            target_pos = target.position + np.random.normal(0, 0.1, 2)
            
            # Calculate power based on distance
            power = 0.5
            if pass_dist > 3.0:
                power = 0.7
            elif pass_dist > 1.5:
                power = 0.6
                
            # CRITICAL FIX: Store reference to previous owner
            ball = self.player.ball
            ball.last_owner = self.player
            
            # CRITICAL FIX: First clear ownership
            self.player.has_ball = False
            ball.owner = None
            
            # CRITICAL FIX: Set pass cooldown
            ball.pass_cooldown = 0.3  # 0.3 seconds cooldown
            
            # Apply velocity to ball AFTER ownership cleared
            ball.velocity = pass_dir * (power * ball.kick_power_factor)
            
            print(f"Pass executed: {self.player.role} to {target.role}, power={power:.1f}")
            
            # Change state
            self.player.fsm.change_state(PlayerState.POSITIONING)
            return True
            
        except Exception as e:
            print(f"ERROR in pass execution: {e}")
            return False
            
    def shoot_at_goal(self):
        """Shoot the ball at goal"""
        # Calculate basic shot parameters
        goal_pos = self.player.opponent_goal
        
        # Add randomization to aim (don't always shoot center)
        goal_width = 2.6
        offset = np.random.uniform(-goal_width/3, goal_width/3)
        
        # Perpendicular direction to goal line
        perp = np.array([0, 1]) if self.player.team.team_side == 'home' else np.array([0, -1])
        
        # Target position within goal
        target_pos = goal_pos + perp * offset
        
        # Direction to target
        shot_vector = target_pos - self.player.position
        shot_dist = np.linalg.norm(shot_vector)
        
        if shot_dist > 0:
            shot_dir = shot_vector / shot_dist
            
            # Calculate power based on distance
            power = 0.8 + (shot_dist * 0.1)  # More power for longer shots
            power = min(power, 1.2)  # Cap at 1.2
            
            # CRITICAL FIX: Release ball before applying velocity
            self.player.has_ball = False
            self.player.ball.owner = None
            
            # Execute shot
            self.player.ball.kick(shot_dir, power)
            print(f"{self.player.role} shoots at goal! Power={power:.1f}")
            
            # Change state
            self.player.fsm.change_state(PlayerState.POSITIONING)
            
    def clear_ball(self):
        """Clear the ball upfield"""
        # Direction is primarily forward
        if self.player.team.team_side == 'home':
            forward_dir = np.array([1.0, 0.0])
        else:
            forward_dir = np.array([-1.0, 0.0])
            
        # Add randomization to avoid predictability
        random_y = np.random.uniform(-0.3, 0.3)
        clear_dir = forward_dir + np.array([0, random_y])
        clear_dir = clear_dir / np.linalg.norm(clear_dir)
        
        # CRITICAL FIX: Release ball before applying velocity
        self.player.has_ball = False
        self.player.ball.owner = None
        
        # Execute clearance with high power
        self.player.ball.kick(clear_dir, 0.9)
        print(f"{self.player.role} clears the ball!")
        
        # Change state
        self.player.fsm.change_state(PlayerState.POSITIONING)
        
    def change_dribble_direction(self):
        """Update the dribble direction with controlled randomness"""
        self.direction_change_timer = 0
        
        # Get base forward direction based on team
        if self.player.team.team_side == 'home':
            base_forward = np.array([1.0, 0.0])
        else:
            base_forward = np.array([-1.0, 0.0])
            
        # Get direction to goal
        goal_dir = self.player.opponent_goal - self.player.position
        if np.linalg.norm(goal_dir) > 0:
            goal_dir = goal_dir / np.linalg.norm(goal_dir)
        else:
            goal_dir = base_forward
            
        # Random lateral component
        random_y = np.random.uniform(-0.5, 0.5)
        lateral_dir = np.array([0, random_y])
        
        # Role-based directional preferences
        if self.player.role.startswith('striker'):
            # Strikers more goal-oriented
            weight_goal = 0.7
            weight_lateral = 0.3
        elif self.player.role == 'midfielder':
            # Midfielders balanced
            weight_goal = 0.5
            weight_lateral = 0.5
        else:
            # Defenders - check if in opponent half for more aggressive posture
            is_in_opponent_half = False
            if self.player.position[0] > 0 and self.player.team.team_side == 'home':
                is_in_opponent_half = True
            elif self.player.position[0] < 0 and self.player.team.team_side == 'away':
                is_in_opponent_half = True
                
            if is_in_opponent_half:
                # More aggressive when forward
                weight_goal = 0.5
                weight_lateral = 0.5
            else:
                # More cautious in own half
                weight_goal = 0.3
                weight_lateral = 0.7
            
        # Combine directions
        new_dir = goal_dir * weight_goal + lateral_dir * weight_lateral
        
        # Normalize
        if np.linalg.norm(new_dir) > 0:
            new_dir = new_dir / np.linalg.norm(new_dir)
            
        # Set new direction
        self.dribble_direction = new_dir
        print(f"DEBUG: {self.player.role} changed direction to {self.dribble_direction}")
        
    def dribble_with_direction(self, dt, goal_bias=0.8):
        """Dribble using current direction with some goal influence"""
        if self.dribble_direction is None:
            self.change_dribble_direction()
            
        # Current position
        current_pos = self.player.position
        
        # Mix current direction with goal direction
        goal_dir = self.player.opponent_goal - current_pos
        if np.linalg.norm(goal_dir) > 0:
            goal_dir = goal_dir / np.linalg.norm(goal_dir)
        else:
            # Fallback
            goal_dir = np.array([1.0, 0.0]) if self.player.team.team_side == 'home' else np.array([-1.0, 0.0])
        
        # Combine directions
        combined_dir = self.dribble_direction * (1 - goal_bias) + goal_dir * goal_bias
        
        # Normalize
        if np.linalg.norm(combined_dir) > 0:
            combined_dir = combined_dir / np.linalg.norm(combined_dir)
        
        # Apply obstacle avoidance for close opponents
        for opponent in self.player.opponents:
            to_opponent = opponent.position - current_pos
            dist = np.linalg.norm(to_opponent)
            
            # Only avoid very close opponents
            if dist < 1.0:
                if dist > 0:
                    avoid_dir = -to_opponent / dist
                    avoid_weight = max(0, 1.0 - dist) / 1.0
                    
                    # Reduce avoidance for strikers to be more aggressive
                    if self.player.role.startswith('striker'):
                        avoid_weight *= 0.5
                    # ENHANCED: Also reduce avoidance for defenders to be more challenging
                    elif 'defender' in self.player.role:
                        avoid_weight *= 0.7
                        
                    combined_dir = combined_dir * (1 - avoid_weight) + avoid_dir * avoid_weight
                    
                    # Renormalize
                    if np.linalg.norm(combined_dir) > 0:
                        combined_dir = combined_dir / np.linalg.norm(combined_dir)
        
        # Target position - move in combined direction
        target_pos = current_pos + combined_dir * 0.6
        
        # Ensure we stay in our zone
        zone = self.player.team.strategy.zones.get(self.player.role, {})
        if zone:
            x = min(max(target_pos[0], zone.get("min_x", -5.0) + 0.1), zone.get("max_x", 5.0) - 0.1)
            y = min(max(target_pos[1], zone.get("min_y", -3.0) + 0.1), zone.get("max_y", 3.0) - 0.1)
            target_pos = np.array([x, y])
        
        # Move toward target
        seek(self.player, target_pos, dt)

class PlayerFSM:
    def __init__(self, player):
        self.player = player
        self.states = {
            PlayerState.POSITIONING: PositioningState(player),
            PlayerState.PURSUE_BALL: PursueBallState(player),
            PlayerState.POSSESSION: PossessionState(player)
        }
        self.current_state = self.states[PlayerState.POSITIONING]

    def update(self, dt):
        if self.current_state:
            self.current_state.execute(dt)

    def change_state(self, new_state):
        if isinstance(new_state, str):
            new_state = PlayerState(new_state)
        
        if self.current_state != self.states[new_state]:
            if self.current_state:
                self.current_state.exit()
            self.current_state = self.states[new_state]
            self.current_state.enter()