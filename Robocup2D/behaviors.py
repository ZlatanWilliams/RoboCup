import math
from game_objects import Ball

class Strategy:
    def __init__(self, team_side):
        self.team_side = team_side  # 'left' or 'right'
        self.roles_assigned = False
        
    def assign_roles(self, players):
        """Assign roles to players based on strategy"""
        raise NotImplementedError
        
    def update(self, players, ball, opponent_players):
        """Update all players' behavior"""
        raise NotImplementedError

class BalancedStrategy(Strategy):
    def __init__(self, team_side):
        super().__init__(team_side)
        self.formation = {
            'goalkeeper': 1,
            'defender': 1,
            'striker': 1,
            'midfielder': 1
        }
        
    def assign_roles(self, players):
        """Assign roles to players based on balanced formation"""
        if len(players) != 4:
            raise ValueError("Balanced strategy requires exactly 4 players")
            
        players[0].role = 'goalkeeper'
        players[1].role = 'defender'
        players[2].role = 'striker'
        players[3].role = 'midfielder'
        self.roles_assigned = True
        
    def update(self, players, ball, opponent_players):
        """Update player behaviors based on their roles"""
        for player in players:
            if player.role == 'goalkeeper':
                self._update_goalkeeper(player, ball)
            elif player.role == 'defender':
                self._update_defender(player, ball, opponent_players)
            elif player.role == 'striker':
                self._update_striker(player, ball)
            elif player.role == 'midfielder':
                self._update_midfielder(player, ball, opponent_players)

    def _update_goalkeeper(self, goalkeeper, ball):
        """Goalkeeper behavior with ball tracking"""
        # Goal boundaries calculation
        goal_center_y = 0
        goal_width = 2.6  # From pitch specifications
        max_y = goal_width/2 - 0.3  # Keep some margin
        
        # Track ball Y position but stay within goal
        target_y = ball.position[1]
        target_y = max(min(target_y, max_y), -max_y)
        
        # Determine X position based on team side
        if self.team_side == 'left':
            target_x = -4.2  # From previous initialization
        else:
            target_x = 4.2
            
        goalkeeper.target_position = (target_x, target_y)
        self._move_towards(goalkeeper, goalkeeper.target_position)

    def _update_defender(self, defender, ball, opponents):
        """Defender behavior with zone protection"""
        # Stay in defensive half
        if self.team_side == 'left':
            max_x = 0  # Don't cross halfway line
            penalty_area_front = -3.5  # Adjust based on pitch dimensions
        else:
            max_x = 0
            penalty_area_front = 3.5
            
        # If ball in defensive zone, intercept
        if (self._is_in_defensive_half(ball.position) and 
            self._distance(defender.position, ball.position) < 2):
            self._move_towards(defender, ball.position)
            if self._distance(defender.position, ball.position) < 0.5:
                self._take_ball(defender, ball)
        else:
            # Patrol penalty area front
            target_x = penalty_area_front
            target_y = ball.position[1] * 0.7  # Track ball y slightly
            defender.target_position = (target_x, target_y)
            self._move_towards(defender, defender.target_position)

    def _update_striker(self, striker, ball):
        """Striker behavior with offensive positioning"""
        # Stay in opponent's half
        if self.team_side == 'left':
            min_x = 0  # Cross halfway line
            target_area = 3.5  # Opponent's penalty area
        else:
            min_x = 0
            target_area = -3.5
            
        # Move towards scoring position
        target_x = target_area
        target_y = ball.position[1]
        striker.target_position = (target_x, target_y)
        
        # If close to ball, attempt to shoot
        if self._distance(striker.position, ball.position) < 1:
            self._move_towards(striker, ball.position)
            if self._distance(striker.position, ball.position) < 0.5:
                self._shoot(striker, ball)
        else:
            self._move_towards(striker, striker.target_position)

    def _update_midfielder(self, midfielder, ball, opponents):
        """Midfielder dual-role behavior"""
        if self._is_in_defensive_half(ball.position):
            # Defensive mode - track opponent striker
            opponent_striker = next(p for p in opponents if p.role == 'striker')
            self._move_towards(midfielder, opponent_striker.position)
            if self._distance(midfielder.position, ball.position) < 0.5:
                self._take_ball(midfielder, ball)
        else:
            # Offensive mode - support striker
            if ball.owner == midfielder:
                team_striker = next(p for p in midfielder.team if p.role == 'striker')
                self._pass_ball(midfielder, team_striker)
            else:
                self._move_towards(midfielder, ball.position)

    # Helper methods
    def _move_towards(self, player, target, speed=0.1):
        """Basic movement towards target position"""
        dx = target[0] - player.position[0]
        dy = target[1] - player.position[1]
        distance = math.hypot(dx, dy)
        
        if distance > 0:
            player.position = (
                player.position[0] + (dx/distance) * speed,
                player.position[1] + (dy/distance) * speed
            )

    def _distance(self, pos1, pos2):
        return math.hypot(pos1[0]-pos2[0], pos1[1]-pos2[1])

    def _is_in_defensive_half(self, position):
        if self.team_side == 'left':
            return position[0] < 0
        return position[0] > 0

    def _take_ball(self, player, ball):
        if self._distance(player.position, ball.position) < 0.5:
            ball.owner = player
            # Move ball with player
            ball.position = player.position

    def _pass_ball(self, from_player, to_player):
        if ball.owner == from_player:
            direction = math.degrees(math.atan2(
                to_player.position[1] - from_player.position[1],
                to_player.position[0] - from_player.position[0]
            ))
            ball.kick(force=3.0, direction=direction)
            ball.owner = None

    def _shoot(self, player, ball):
        if self.team_side == 'left':
            goal_target = (4.5, 0)  # Right goal center
        else:
            goal_target = (-4.5, 0)  # Left goal center
            
        direction = math.degrees(math.atan2(
            goal_target[1] - player.position[1],
            goal_target[0] - player.position[0]
        ))
        ball.kick(force=5.0, direction=direction)
        ball.owner = None