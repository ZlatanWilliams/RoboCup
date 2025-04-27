import numpy as np
from behaviors import BalancedStrategy

class DefensiveStrategy:
    def __init__(self, team_side):
        self.team_side = team_side
        
        # Define zones with proper constraints and different areas for defenders
        if self.team_side == 'home':
            self.zones = {
                'goalkeeper': {"min_x": -4.5, "max_x": -3.8, "min_y": -1.0, "max_y": 1.0},
                # Defender1 focuses more on central coverage (sweeper)
                'defender1': {"min_x": -3.8, "max_x": -1.5, "min_y": -2.0, "max_y": 2.0},
                # Defender2 focuses more on wide coverage (wing defender)
                'defender2': {"min_x": -3.8, "max_x": -2.0, "min_y": -3.0, "max_y": 3.0},
                'striker': {"min_x": -2.0, "max_x": 4.0, "min_y": -2.5, "max_y": 2.5}
            }
        else:  # away team
            self.zones = {
                'goalkeeper': {"min_x": 3.8, "max_x": 4.5, "min_y": -1.0, "max_y": 1.0},
                # Defender1 focuses more on central coverage (sweeper)
                'defender1': {"min_x": 1.5, "max_x": 3.8, "min_y": -2.0, "max_y": 2.0},
                # Defender2 focuses more on wide coverage (wing defender)
                'defender2': {"min_x": 2.0, "max_x": 3.8, "min_y": -3.0, "max_y": 3.0},
                'striker': {"min_x": -4.0, "max_x": 2.0, "min_y": -2.5, "max_y": 2.5}
            }
        
        # Initialize last ball position for calculating ball movement
        self.last_ball_pos = np.array([0.0, 0.0])

    def update_player_positions(self, players, ball_pos):
        try:
            ball_pos = np.array(ball_pos, dtype=float)
            
            # Calculate ball movement direction
            ball_movement = ball_pos - self.last_ball_pos
            self.last_ball_pos = ball_pos.copy()
            
            # Determine if ball is in our half
            is_ball_in_our_half = (ball_pos[0] < 0 if self.team_side == 'home' 
                                 else ball_pos[0] > 0)
            
            # Determine if ball is moving toward our goal
            is_ball_approaching = False
            if np.linalg.norm(ball_movement) > 0.05:  # Only consider significant movement
                if (self.team_side == 'home' and ball_movement[0] < 0) or \
                   (self.team_side == 'away' and ball_movement[0] > 0):
                    is_ball_approaching = True
            
            # Determine which side of the field the ball is on
            ball_is_on_lower_half = ball_pos[1] < 0
            
            for player in players:
                if player.role == 'goalkeeper':
                    # Goalkeeper stays on the goal line, only moving vertically
                    gk_x = -4.3 if self.team_side == 'home' else 4.3
                    raw_target = np.array([gk_x, np.clip(ball_pos[1] * 0.7, -1.0, 1.0)])
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])

                elif player.role == 'defender1':
                    # CENTRAL DEFENDER (SWEEPER) - Stays centrally and protects the goal
                    # Positions closer to goal and responds more directly to ball y-position
                    if is_ball_in_our_half:
                        # Stay closer to goal when ball is in our half
                        if self.team_side == 'home':
                            def_x = -3.2  # Deeper position
                        else:
                            def_x = 3.2
                    else:
                        # Move slightly forward when ball is in opponent half
                        if self.team_side == 'home':
                            def_x = -2.8
                        else:
                            def_x = 2.8
                    
                    # Central defender follows ball y-position more closely
                    ball_follow_weight = 0.8
                    raw_target = np.array([def_x, ball_pos[1] * ball_follow_weight])
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])
                    print(f"Sweeper position updated: {player.target_position}")

                elif player.role == 'defender2':
                    # WIDE DEFENDER - Focuses on covering wide areas and moving forward
                    # Positions wider and is more aggressive in coming forward
                    def_x_base = -2.8 if self.team_side == 'home' else 2.8
                    
                    # More aggressive depending on ball position
                    if not is_ball_in_our_half:
                        # More forward when ball is in opponent half
                        def_x = def_x_base + 0.7 if self.team_side == 'home' else def_x_base - 0.7
                    elif is_ball_approaching:
                        # Deeper when ball is approaching our goal
                        def_x = def_x_base - 0.3 if self.team_side == 'home' else def_x_base + 0.3
                    else:
                        def_x = def_x_base
                    
                    # Wide defender covers wide area based on ball position
                    # Move to same half as ball but with wider positioning
                    if ball_is_on_lower_half:
                        # Position on lower half but wider
                        raw_target = np.array([def_x, min(-0.5, ball_pos[1] - 0.5)])
                    else:
                        # Position on upper half but wider
                        raw_target = np.array([def_x, max(0.5, ball_pos[1] + 0.5)])
                    
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])
                    print(f"Wide defender position updated: {player.target_position}")

                elif player.role == 'striker':
                    # Striker stays forward for counter-attacks
                    if is_ball_in_our_half:
                        # More defensive position when defending
                        str_x = -1.0 if self.team_side == 'home' else 1.0
                        # Position slightly away from ball to find space
                        str_y = ball_pos[1] * 0.3  # Less tracking of ball position
                    else:
                        # More attacking position when in opponent half
                        str_x = 1.0 if self.team_side == 'home' else -1.0
                        # Follow ball position more closely when attacking
                        str_y = ball_pos[1] * 0.7
                    
                    raw_target = np.array([str_x, str_y])
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])

        except Exception as e:
            print(f"Error in update_player_positions: {e}")

    def _clamp_to_zone(self, pos, zone):
        """Clamps a position within the given zone"""
        x_clamped = min(max(pos[0], zone["min_x"]), zone["max_x"])
        y_clamped = min(max(pos[1], zone["min_y"]), zone["max_y"])
        return np.array([x_clamped, y_clamped], dtype=float)

class OffensiveStrategy:
    def __init__(self, team_side):
        self.team_side = team_side
        
        # Define zones with proper constraints
        if self.team_side == 'home':
            self.zones = {
                'goalkeeper': {"min_x": -4.5, "max_x": -3.8, "min_y": -1.0, "max_y": 1.0},
                'defender': {"min_x": -3.8, "max_x": -1.0, "min_y": -2.5, "max_y": 2.5},
                'striker1': {"min_x": -1.0, "max_x": 4.0, "min_y": -2.5, "max_y": 0.0},
                'striker2': {"min_x": -1.0, "max_x": 4.0, "min_y": 0.0, "max_y": 2.5}
            }
        else:  # away team
            self.zones = {
                'goalkeeper': {"min_x": 3.8, "max_x": 4.5, "min_y": -1.0, "max_y": 1.0},
                'defender': {"min_x": 1.0, "max_x": 3.8, "min_y": -2.5, "max_y": 2.5},
                'striker1': {"min_x": -4.0, "max_x": 1.0, "min_y": -2.5, "max_y": 0.0},
                'striker2': {"min_x": -4.0, "max_x": 1.0, "min_y": 0.0, "max_y": 2.5}
            }

    def update_player_positions(self, players, ball_pos):
        try:
            ball_pos = np.array(ball_pos, dtype=float)
            is_ball_in_our_half = (ball_pos[0] < 0 if self.team_side == 'home' 
                                 else ball_pos[0] > 0)
            
            for player in players:
                if player.role == 'goalkeeper':
                    # Goalkeeper stays on the goal line, only moving vertically
                    gk_x = -4.3 if self.team_side == 'home' else 4.3
                    raw_target = np.array([gk_x, np.clip(ball_pos[1] * 0.7, -1.0, 1.0)])
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])

                elif player.role == 'defender':
                    # Defender plays more aggressively, moving up with the attack
                    if is_ball_in_our_half:
                        def_x = -2.5 if self.team_side == 'home' else 2.5
                    else:
                        def_x = -1.5 if self.team_side == 'home' else 1.5
                    raw_target = np.array([def_x, ball_pos[1]])
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])

                elif player.role == 'striker1':
                    # First striker stays in attacking position, lower half
                    if is_ball_in_our_half:
                        str_x = 0.0 if self.team_side == 'home' else 0.0
                    else:
                        # More aggressive forward positioning 
                        str_x = 3.0 if self.team_side == 'home' else -3.0
                    raw_target = np.array([str_x, np.clip(ball_pos[1], -2.5, 0.0)])
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])

                elif player.role == 'striker2':
                    # Second striker provides width in attack, upper half
                    if is_ball_in_our_half:
                        str_x = 0.5 if self.team_side == 'home' else -0.5
                    else:
                        # More aggressive forward positioning
                        str_x = 3.5 if self.team_side == 'home' else -3.5
                    raw_target = np.array([str_x, np.clip(ball_pos[1], 0.0, 2.5)])
                    player.target_position = self._clamp_to_zone(raw_target, self.zones[player.role])

        except Exception as e:
            print(f"Error in update_player_positions: {e}")

    def _clamp_to_zone(self, pos, zone):
        """Clamps a position within the given zone"""
        x_clamped = min(max(pos[0], zone["min_x"]), zone["max_x"])
        y_clamped = min(max(pos[1], zone["min_y"]), zone["max_y"])
        return np.array([x_clamped, y_clamped], dtype=float)