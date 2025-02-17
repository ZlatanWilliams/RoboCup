import math
import random

class FSMState:
    def __init__(self, player):
        self.player = player

    def enter(self):
        pass

    def execute(self):
        pass

    def exit(self):
        pass

# ---------------------- Steering Behaviors ----------------------
def seek(player, target):
    """ Moves player towards a target position """
    tx, ty = target
    px, py = player.position
    
    direction = (tx - px, ty - py)
    distance = math.hypot(direction[0], direction[1])
    
    if distance > 0:
        norm_dir = (direction[0] / distance, direction[1] / distance)
        player.position = (px + norm_dir[0] * player.speed, py + norm_dir[1] * player.speed)

def arrive(player, target, slow_radius=1.0):
    """ Slows down player when reaching a target """
    tx, ty = target
    px, py = player.position
    distance = math.hypot(tx - px, ty - py)
    
    if distance < slow_radius:
        player.speed *= 0.5  # Slow down
    seek(player, target)

def pursue(player, ball):
    """ Predicts ball position and moves toward it """
    future_x = ball.position[0] + ball.velocity[0] * 0.5
    future_y = ball.position[1] + ball.velocity[1] * 0.5
    seek(player, (future_x, future_y))

# ---------------------- Player FSM States ----------------------
class GoalkeeperState(FSMState):
    def __init__(self, player):
        super().__init__(player)
        self.player = player 
    def execute(self):
        # Goalkeeper stays within goal area and blocks shots
        goal_x = -4.5 if self.player.team == 'left' else 4.5
        target_pos = (goal_x, self.player.position[1])
        arrive(self.player, target_pos)

class DefenderState(FSMState):
    def __init__(self, player):
        super().__init__(player)
        self.player = player 
    def execute(self):
        # Defender tracks opponent and blocks
        if self.player.opponent_with_ball:
            seek(self.player, self.player.opponent_with_ball.position)
        else:
            self.player.hold_position()

class MidfielderState(FSMState):
    def __init__(self, player):
        super().__init__(player)
        self.player = player 
    def execute(self):
        # Midfielder moves into space and passes
        if self.player.has_ball:
            best_teammate = self.player.find_best_pass()
            if best_teammate:
                self.player.pass_ball(best_teammate)
            else:
                seek(self.player, self.player.team_goal)
        else:
            pursue(self.player, self.player.ball)

class StrikerState(FSMState):
    def __init__(self, player):
        super().__init__(player)
        self.player = player 
    def execute(self):
        # Striker moves towards goal and shoots
        if self.player.has_ball:
            seek(self.player, self.player.opponent_goal)
            if self.player.is_in_shooting_position():
                self.player.shoot()
        else:
            pursue(self.player, self.player.ball)

class PossessionState(FSMState):
    def __init__(self, player):
        super().__init__(player)
        self.player = player 
    def execute(self):
        if self.player.role == "midfielder":
            best_teammate = self.player.find_best_pass()
            if best_teammate:
                self.player.pass_ball(best_teammate)
            else:
                seek(self.player, self.player.team_goal)
        elif self.player.role == "striker":
            if self.player.is_in_shooting_position():
                self.player.shoot()
            else:
                seek(self.player, self.player.opponent_goal)
        else:
            seek(self.player, self.team_goal)  # Default behavior for other roles

# ---------------------- Player FSM Implementation ----------------------
class PlayerFSM:
    def __init__(self, player):
        self.player = player
        self.states = {
            'goalkeeper': GoalkeeperState(player),
            'defender': DefenderState(player),
            'midfielder': MidfielderState(player),
            'striker': StrikerState(player),
            'possession': PossessionState(player)
        }
        self.current_state = self.states[player.role]

    def update(self):
        self.current_state.execute()

    def change_state(self, new_state_name):
        self.current_state.exit()
        self.current_state = self.states[new_state_name]
        self.current_state.enter()

        # If player gains possession, decide what to do
        if new_state_name == "possession":
            if self.player.role == "midfielder":
                self.player.pass_ball(self.player.find_best_pass())
            elif self.player.role == "striker":
                if self.player.is_in_shooting_position():
                    self.player.shoot()
                else:
                    self.player.dribble()

# ---------------------- Integrating FSM with Players ----------------------
class Player:
    def __init__(self, team, role, initial_position):
        self.team = team
        self.role = role
        self.position = initial_position
        self.speed = 0.5
        self.fsm = PlayerFSM(self)
        self.has_ball = False
        self.ball = None
        self.teammates = []
        self.opponent_goal = (4.5, 0) if team == 'left' else (-4.5, 0)
        self.team_goal = (-4.5, 0) if team == 'left' else (4.5, 0)

    def update(self):
        self.fsm.update()

    def find_best_pass(self):
        # Placeholder logic for passing decision
        return random.choice(self.teammates) if self.teammates else None

    def pass_ball(self, teammate):
        print(f"{self.role} passes the ball to {teammate.role}")
        teammate.has_ball = True
        self.has_ball = False

    def is_in_shooting_position(self):
        return math.hypot(self.position[0] - self.opponent_goal[0], self.position[1] - self.opponent_goal[1]) < 1.5
    
    def shoot(self):
        print(f"{self.role} shoots towards goal!")



# Example Usage
if __name__ == "__main__":
    striker = Player('left', 'striker', (-1.0, 0))
    midfielder = Player('left', 'midfielder', (-2.0, 0))
    striker.teammates.append(midfielder)
    striker.has_ball = True
    
    for _ in range(10):
        striker.update()

