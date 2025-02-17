import pygame
import numpy as np
from behaviors import BalancedStrategy
from fsm import PlayerFSM

class Player:
    def __init__(self, team, role, x, y, color):
        self.team = team
        self.role = role
        self.position = np.array([x, y], dtype=float)
        self.target_position = self.position.copy()  # Add this
        self.color = color
        self.size = 0.15
        self.max_speed = 2.0
        self.speed = 2.0
        self.radius = 0.2
        self.opponent_with_ball = None  # Tracks the opponent who has the ball
        # self.hold_position = None
        self.fsm = PlayerFSM(self)

    def update(self, dt):
        self.fsm.update()
        direction = self.target_position - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:
            movement = (direction / distance) * self.max_speed * dt
            if np.linalg.norm(movement) > distance:
                self.position = self.target_position
            else:
                self.position += movement

    def hold_position(self):
        """Keeps the player in the same position (defensive stance)."""
        self.target_position = self.position.copy()
    def hold_position(self):
        """Keeps the player in the same position (defensive stance)."""
        self.target_position = self.position.copy()
    def get_visual_properties(self):
        return {
            'position': self.position,
            'color': self.color,
            'size': self.size,
            'label': f"{self.team[0].upper()}-{self.role[0].upper()}"
        }

class Team:
    def __init__(self, team_side):
        self.team_side = team_side  # 'home' or 'away'
        self.color = (255, 50, 50) if team_side == 'home' else (50, 50, 255)
        self.players = self.initialize_players()
        self.strategy = BalancedStrategy(team_side)


    def update(self, dt, ball_pos):
        self.strategy.update_player_positions(self.players, ball_pos)
        for player in self.players:
            player.update(dt)
    
    def initialize_players(self):
        if self.team_side == 'home':
            return [
                # Home team (left side)
                Player(self.team_side, 'goalkeeper', -4.0, 0, self.color),      
                Player(self.team_side, 'defender', -2.5, 0, self.color),        
                Player(self.team_side, 'midfielder', -1.0, 1.5, self.color),    
                Player(self.team_side, 'striker', -0.5, -1.5, self.color)     
            ]
        else:
            return [
                # Away team (right side)
                Player(self.team_side, 'goalkeeper', 4.0, 0, self.color),      
                Player(self.team_side, 'defender', 2.5, 0, self.color),    
                Player(self.team_side, 'midfielder', 1.0, -1.5, self.color), 
                Player(self.team_side, 'striker', 0.5, 1.5, self.color)
            ]