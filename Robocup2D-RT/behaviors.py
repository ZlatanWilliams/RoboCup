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

class BalancedStrategy:
    def __init__(self, team_side):
        self.team_side = team_side  # 'home' or 'away'
        self.field_direction = -1 if team_side == 'home' else 1
        

        self.setup_boundaries()
        
    def setup_boundaries(self):

        self.half_length = 4.5  
        self.half_width = 3.0 
        self.goal_width = 1.3 
        

        self.goal_line = -self.half_length if self.team_side == 'home' else self.half_length
        self.goalkeeper_range = 1.5 
        

        self.defender_min_x = -self.half_length if self.team_side == 'home' else 0
        self.defender_max_x = 0 if self.team_side == 'home' else self.half_length
        

        self.striker_min_x = 0 if self.team_side == 'home' else -self.half_length
        self.striker_max_x = self.half_length if self.team_side == 'home' else 0

    def get_goalkeeper_position(self, ball_pos, current_pos):
        x_pos = self.goal_line
        ball_y = ball_pos[1]
        y_pos = np.clip(ball_y, -self.goal_width, self.goal_width)
        ball_distance = abs(ball_pos[0] - self.goal_line)
        if ball_distance < self.goalkeeper_range:
            x_pos += self.field_direction * (self.goalkeeper_range - ball_distance)
            
        return np.array([x_pos, y_pos])

    def get_defender_position(self, ball_pos, current_pos):
        defender_x = np.clip(
            ball_pos[0],
            self.defender_min_x,
            self.defender_max_x
        )
        defender_y = np.clip(ball_pos[1], -self.half_width + 0.5, self.half_width - 0.5)
        
        return np.array([defender_x, defender_y])

    def get_striker_position(self, ball_pos, current_pos):
        if self.is_ball_in_striker_area(ball_pos):
            target_x = np.clip(
                ball_pos[0],
                self.striker_min_x,
                self.striker_max_x
            )
            target_y = ball_pos[1]
        else:
            target_x = self.striker_min_x + (self.field_direction * 3)
            target_y = np.clip(ball_pos[1], -2, 2)
            
        return np.array([target_x, target_y])

    def get_midfielder_position(self, ball_pos, current_pos):
        in_defensive_half = (ball_pos[0] < 0 and self.team_side == 'home') or \
                          (ball_pos[0] > 0 and self.team_side == 'away')
        
        if in_defensive_half:
            target_x = ball_pos[0] - (self.field_direction * 1)
            target_y = ball_pos[1]
        else:
            target_x = ball_pos[0] + (self.field_direction * 1)
            target_y = -ball_pos[1]
        target_x = np.clip(target_x, -self.half_length + 0.5, self.half_length - 0.5)
        target_y = np.clip(target_y, -self.half_width + 0.5, self.half_width - 0.5)
        
        return np.array([target_x, target_y])

    def is_ball_in_striker_area(self, ball_pos):
        if self.team_side == 'home':
            return ball_pos[0] > 0
        else:
            return ball_pos[0] < 0

    def update_player_positions(self, players, ball_pos):
        for player in players:
            current_pos = player.position
            
            if player.role == PlayerRole.GOALKEEPER.value:
                target_pos = self.get_goalkeeper_position(ball_pos, current_pos)
            elif player.role == PlayerRole.DEFENDER.value:
                target_pos = self.get_defender_position(ball_pos, current_pos)
            elif player.role == PlayerRole.STRIKER.value:
                target_pos = self.get_striker_position(ball_pos, current_pos)
            elif player.role == PlayerRole.MIDFIELDER.value:
                target_pos = self.get_midfielder_position(ball_pos, current_pos)

            player.target_position = target_pos