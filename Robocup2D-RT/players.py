import pygame
import numpy as np
from behaviors import BalancedStrategy
from fsm import PlayerFSM

class Player:
    def __init__(self, team, role, x, y, color):
        self.team_name = team  # "home" or "away"
        self.team = None  # This will be assigned later by Team class
        self.role = role
        self.position = np.array([x, y], dtype=float)
        self.target_position = self.position.copy()  
        self.color = color
        self.size = 0.15
        self.max_speed = 2.0
        self.speed = 2.0
        self.radius = 0.2
        self.has_ball = False  # ✅ Tracks if player has possession
        self.opponent_with_ball = None  # ✅ Tracks the opponent who has the ball
        self.fsm = PlayerFSM(self)  # ✅ Assign FSM for decision-making

        # ✅ Define goal positions based on team
        self.opponent_goal = np.array([4.5, 0]) if team == 'home' else np.array([-4.5, 0])
        self.team_goal = np.array([-4.5, 0]) if team == 'home' else np.array([4.5, 0])

    def update(self, dt):
        """Updates the player state using FSM and movement."""
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

    def get_visual_properties(self):
        """Returns properties used for visualization in pygame."""
        return {
            'position': self.position,
            'color': self.color,
            'size': self.size,
            'label': f"{self.team_name[0].upper()}-{self.role[0].upper()}"
        }
    
    def pass_ball(self, teammate):
        """Passes the ball to a teammate."""
        if teammate:
            print(f"⚽ {self.role} passes the ball to {teammate.role}")
            self.has_ball = False
            teammate.has_ball = True
            if hasattr(self, 'ball'):
                teammate.ball = self.ball  # ✅ Assign ball ownership
                teammate.ball.owner = teammate
                teammate.ball.velocity = (np.array(teammate.position) - self.position) * 0.8  # ✅ Adjust pass speed

    def dribble(self):
        """Dribbles the ball towards the opponent's goal."""
        if self.has_ball:
            print(f"⚽ {self.role} is dribbling towards the goal!")
            direction = self.opponent_goal - self.position
            distance = np.linalg.norm(direction)

            if distance > 0.1:
                movement = (direction / distance) * self.max_speed * 0.5  # ✅ Slightly slower than running
                self.position += movement

                # ✅ Ball moves along with the player
                if hasattr(self, 'ball'):
                    self.ball.position = self.position.copy()
                    self.ball.velocity = np.array([0.0, 0.0])

    def find_best_pass(self):
        """Finds the best teammate to pass to (placeholder logic)."""
        if not self.team:
            return None
        
        best_teammate = None
        min_distance = float('inf')
        for teammate in self.team.players:
            if teammate != self and not teammate.has_ball:
                distance = np.linalg.norm(np.array(teammate.position) - self.position)
                if distance < min_distance:
                    min_distance = distance
                    best_teammate = teammate
                    
        return best_teammate

    def is_in_shooting_position(self):
        """Determines if the player is in a good position to shoot at the goal."""
        goal_x = 4.5 if self.team_name == "home" else -4.5  # Opponent's goal position
        distance_to_goal = np.linalg.norm(self.position - np.array([goal_x, 0]))

        print(f"⚽ Debug: {self.role} checking shooting position: Distance to goal = {distance_to_goal}")

        return distance_to_goal < 1.5  # ✅ Adjust shooting threshold

    def shoot(self):
        """Executes a shot toward the opponent's goal."""
        if hasattr(self, 'ball') and self.has_ball:
            print(f"⚽ {self.role} shoots towards goal!")
            self.ball.velocity = (self.opponent_goal - self.position) * 1.5  # ✅ Adjust shot power
            self.ball.owner = None  # ✅ Ball is free after shot
            self.has_ball = False  # ✅ Player no longer has possession


class Team:
    def __init__(self, team_side):
        self.team_side = team_side  # 'home' or 'away'
        self.color = (255, 50, 50) if team_side == 'home' else (50, 50, 255)
        self.players = self.initialize_players()
        self.strategy = BalancedStrategy(team_side)

        # ✅ Assign team reference to players
        for player in self.players:
            player.team = self

    def update(self, dt, ball_pos):
        """Updates players' positions based on strategy and FSM."""
        self.strategy.update_player_positions(self.players, ball_pos)
        for player in self.players:
            player.update(dt)
    
    def initialize_players(self):
        if self.team_side == 'home':
            return [
                Player(self.team_side, 'goalkeeper', -4.3, 0, self.color),  # ✅ Move slightly back  
                Player(self.team_side, 'defender', -3.0, 0, self.color),  # ✅ Move outward
                Player(self.team_side, 'midfielder', -1.5, 1.0, self.color),  # ✅ Adjust to central
                Player(self.team_side, 'striker', -0.8, -1.0, self.color)  # ✅ Adjust positioning
            ]
        else:
            return [
                Player(self.team_side, 'goalkeeper', 4.3, 0, self.color),  # ✅ Move slightly back
                Player(self.team_side, 'defender', 3.0, 0, self.color),  # ✅ Move outward
                Player(self.team_side, 'midfielder', 1.5, -1.0, self.color),  # ✅ Adjust to central
                Player(self.team_side, 'striker', 0.8, 1.0, self.color)  # ✅ Adjust positioning
            ]
