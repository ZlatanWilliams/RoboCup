import numpy as np
from enum import Enum

class GamePhase(Enum):
    ATTACK = "attack"
    DEFENSE = "defense"
    TRANSITION = "transition"

class TacticalEngine:
    def __init__(self):
        self.home_danger_zone = {"min_x": -4.5, "max_x": -2.0, "min_y": -2.0, "max_y": 2.0}
        self.away_danger_zone = {"min_x": 2.0, "max_x": 4.5, "min_y": -2.0, "max_y": 2.0}
        
    def analyze_game_state(self, team, ball):
        """Analyze current game situation and return tactical info"""
        ball_pos = np.array(ball.position)
        
        # Determine game phase
        phase = self._determine_phase(team, ball)
        
        # Calculate key tactical positions
        support_positions = self._calculate_support_positions(team, ball_pos, phase)
        
        # Find passing lanes
        passing_lanes = self._analyze_passing_lanes(team, ball)
        
        # Check if in dangerous position (near own goal)
        in_danger = self._check_danger_zone(team, ball_pos)
        
        return {
            'phase': phase,
            'support_positions': support_positions,
            'passing_lanes': passing_lanes,
            'in_danger': in_danger,
            'ball_position': ball_pos
        }
    
    def _determine_phase(self, team, ball):
        """Determine if team is attacking, defending, or in transition"""
        if ball.owner is None:
            return GamePhase.TRANSITION
            
        if ball.owner in team.players:
            return GamePhase.ATTACK
        else:
            return GamePhase.DEFENSE
    
    def _calculate_support_positions(self, team, ball_pos, phase):
        """Calculate optimal positions for supporting players"""
        positions = {}
        is_home = team.team_side == 'home'
        
        if phase == GamePhase.ATTACK:
            # Attacking formation
            if is_home:
                positions = {
                    'goalkeeper': np.array([-4.3, 0.0]),
                    'defender': np.array([-2.5, ball_pos[1] * 0.3]),
                    'midfielder': np.array([ball_pos[0] - 1.0, ball_pos[1]]),
                    'striker': np.array([ball_pos[0] + 1.0, -ball_pos[1]])
                }
            else:
                positions = {
                    'goalkeeper': np.array([4.3, 0.0]),
                    'defender': np.array([2.5, ball_pos[1] * 0.3]),
                    'midfielder': np.array([ball_pos[0] + 1.0, ball_pos[1]]),
                    'striker': np.array([ball_pos[0] - 1.0, -ball_pos[1]])
                }
        else:
            # Defensive formation
            if is_home:
                positions = {
                    'goalkeeper': np.array([-4.3, np.clip(ball_pos[1] * 0.7, -1.0, 1.0)]),
                    'defender': np.array([-3.0, ball_pos[1]]),
                    'midfielder': np.array([-1.0, ball_pos[1] * 0.5]),
                    'striker': np.array([0.0, -ball_pos[1] * 0.3])
                }
            else:
                positions = {
                    'goalkeeper': np.array([4.3, np.clip(ball_pos[1] * 0.7, -1.0, 1.0)]),
                    'defender': np.array([3.0, ball_pos[1]]),
                    'midfielder': np.array([1.0, ball_pos[1] * 0.5]),
                    'striker': np.array([0.0, -ball_pos[1] * 0.3])
                }
        
        return positions
    
    def _analyze_passing_lanes(self, team, ball):
        """Find and rate possible passing lanes"""
        if ball.owner not in team.players:
            return []
            
        lanes = []
        for player in team.players:
            if player != ball.owner:
                # Calculate basic pass score based on position
                direction = player.position - ball.owner.position
                distance = np.linalg.norm(direction)
                
                if distance < 4.0:  # Maximum pass distance
                    # Score based on position and role
                    score = self._calculate_pass_score(team, player, distance)
                    lanes.append((player, score))
        
        # Sort by score
        return sorted(lanes, key=lambda x: x[1], reverse=True)
    
    def _calculate_pass_score(self, team, player, distance):
        """Calculate how good a passing option is"""
        score = 0
        is_home = team.team_side == 'home'
        
        # Base score from distance (closer is better, but not too close)
        score -= (distance - 2.0) ** 2
        
        # Bonus for forward passes
        forward_bonus = player.position[0] - ball.owner.position[0]
        if is_home:
            score += forward_bonus
        else:
            score -= forward_bonus
            
        # Role-based bonuses
        if player.role == 'striker':
            score += 2
        elif player.role == 'midfielder':
            score += 1
            
        return score
    
    def _check_danger_zone(self, team, ball_pos):
        """Check if ball is in team's dangerous area"""
        danger_zone = self.home_danger_zone if team.team_side == 'home' else self.away_danger_zone
        return (danger_zone["min_x"] <= ball_pos[0] <= danger_zone["max_x"] and
                danger_zone["min_y"] <= ball_pos[1] <= danger_zone["max_y"])