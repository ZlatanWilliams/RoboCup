import numpy as np
from enum import Enum

class TeamState(Enum):
    DEFENSIVE = "defensive"
    OFFENSIVE = "offensive"
    BALANCED = "balanced"

class PlayerRole(Enum):
    GOALKEEPER = "goalkeeper"
    DEFENDER = "defender"
    MIDFIELDER = "midfielder"
    STRIKER = "striker"

def clamp_to_zone(pos, zone):
    """Clamps a position within the given zone"""
    x_clamped = min(max(pos[0], zone["min_x"]), zone["max_x"])
    y_clamped = min(max(pos[1], zone["min_y"]), zone["max_y"])
    return np.array([x_clamped, y_clamped], dtype=float)

class BalancedStrategy:
    def __init__(self, team_side):
        self.team_side = team_side
        
        # Define zones with proper constraints
        if self.team_side == 'home':
            self.zones = {
                'goalkeeper': {"min_x": -4.5, "max_x": -3.8, "min_y": -1.0, "max_y": 1.0},
                'defender': {"min_x": -3.8, "max_x": -2.0, "min_y": -2.5, "max_y": 2.5},
                'midfielder': {"min_x": -2.0, "max_x": 1.0, "min_y": -2.5, "max_y": 2.5},
                'striker': {"min_x": 0.0, "max_x": 4.0, "min_y": -2.5, "max_y": 2.5}
            }
        else:  # away team
            self.zones = {
                'goalkeeper': {"min_x": 3.8, "max_x": 4.5, "min_y": -1.0, "max_y": 1.0},
                'defender': {"min_x": 2.0, "max_x": 3.8, "min_y": -2.5, "max_y": 2.5},
                'midfielder': {"min_x": -1.0, "max_x": 2.0, "min_y": -2.5, "max_y": 2.5},
                'striker': {"min_x": -4.0, "max_x": 0.0, "min_y": -2.5, "max_y": 2.5}
            }

    def update_player_positions(self, players, ball_pos):
        """Updates player positions based on ball position and roles"""
        try:
            ball_pos = np.array(ball_pos, dtype=float)
            is_ball_in_our_half = (ball_pos[0] < 0 if self.team_side == 'home' 
                                 else ball_pos[0] > 0)
            
            for player in players:
                if player.role == 'goalkeeper':
                    # Goalkeeper stays on the goal line, only moving vertically
                    gk_x = -4.3 if self.team_side == 'home' else 4.3
                    # Follow ball vertically but stay within small area
                    raw_target = np.array([gk_x, np.clip(ball_pos[1] * 0.7, -1.0, 1.0)])
                    player.target_position = clamp_to_zone(raw_target, self.zones[player.role])

                elif player.role == 'defender':
                    # Defender stays between goal and ball
                    def_x = -3.0 if self.team_side == 'home' else 3.0
                    if is_ball_in_our_half:
                        # Stay closer to goal when ball is in our half
                        raw_target = np.array([def_x, np.clip(ball_pos[1] * 0.8, -2.0, 2.0)])
                    else:
                        # Move up slightly when ball is in opponent's half
                        raw_target = np.array([def_x + 0.5, np.clip(ball_pos[1] * 0.6, -2.0, 2.0)])
                    player.target_position = clamp_to_zone(raw_target, self.zones[player.role])

                elif player.role == 'midfielder':
                    # Midfielder moves to support attack or defense
                    if is_ball_in_our_half:
                        # Support defense
                        mid_x = -1.0 if self.team_side == 'home' else 1.0
                        raw_target = np.array([mid_x, ball_pos[1]])
                    else:
                        # Support attack
                        mid_x = 0.5 if self.team_side == 'home' else -0.5
                        raw_target = np.array([mid_x, ball_pos[1]])
                    player.target_position = clamp_to_zone(raw_target, self.zones[player.role])

                elif player.role == 'striker':
                    # Striker stays forward and tries to find space
                    if is_ball_in_our_half:
                        # Stay somewhat forward when defending
                        str_x = 0.5 if self.team_side == 'home' else -0.5
                        raw_target = np.array([str_x, -ball_pos[1]]) # Move opposite to ball for space
                    else:
                        # Get forward when attacking
                        str_x = 2.0 if self.team_side == 'home' else -2.0
                        raw_target = np.array([str_x, ball_pos[1]])
                    player.target_position = clamp_to_zone(raw_target, self.zones[player.role])

        except Exception as e:
            print(f"Error in update_player_positions: {e}")

class MinimalStrategy:
    def __init__(self, team_side):
        self.balanced_strategy = BalancedStrategy(team_side)
        self.zones = self.balanced_strategy.zones
        self.team_side = team_side

    def update_player_positions(self, players, ball_pos):
        """Minimal strategy also maintains proper formation"""
        self.balanced_strategy.update_player_positions(players, ball_pos)