import pygame
import numpy as np
from fsm import PlayerState
from tactical_engine import TacticalEngine
from physics_engine import PhysicsEngine

class GameEngine:
    def __init__(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.debug_mode = True
        self.score = {"home": 0, "away": 0}
        self.goal_width = 2.6
        self.reset_delay = 2.0
        self.reset_timer = 0
        self.goal_scored = False
        self.event_handlers = []  # List of additional event handlers

    def run(self, pitch, teams, ball):
        self.screen = pitch.initialize_display()
        self.running = True
        while self.running:
            dt = self.clock.tick(60) / 1000.0  # delta time in seconds
            self.handle_events()
            self.update(pitch, teams, ball, dt)
            self.render(pitch, teams, ball)
        pygame.quit()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False

    def update(self, pitch, teams, ball, dt):
        if self.goal_scored:
            self.reset_timer += dt
            if self.reset_timer >= self.reset_delay:
                self.perform_reset(teams, ball)
                self.goal_scored = False
                self.reset_timer = 0
            return

        # Update the ball's physics first
        ball.update(dt)
        
        # Check for goals before constraining ball
        if self.check_for_goal(ball):
            self.goal_scored = True
            return
            
        self.constrain_ball_to_pitch(ball, pitch)
        
        # Update each team
        for team in teams.values():
            # Store current ball position for strategy
            team.update(dt, ball.position)
            
            # Check collisions for each player
            for player in team.players:
                # Set ball reference for each player
                player.ball = ball
                self.check_player_ball_collision(player, teams, ball)

    def check_for_goal(self, ball):
        # Check home goal (left side)
        if ball.position[0] < -4.5 and abs(ball.position[1]) < self.goal_width/2:
            self.score["away"] += 1
            print(f"GOAL for Away team! Score: Home {self.score['home']} - {self.score['away']} Away")
            return True
            
        # Check away goal (right side)
        if ball.position[0] > 4.5 and abs(ball.position[1]) < self.goal_width/2:
            self.score["home"] += 1
            print(f"GOAL for Home team! Score: Home {self.score['home']} - {self.score['away']} Away")
            return True
            
        return False

    def perform_reset(self, teams, ball):
        """Reset all positions after a goal"""
        # Reset ball
        ball.position = np.array([0.0, 0.0])
        ball.velocity = np.array([0.0, 0.0])
        ball.owner = None
        
        # Reset all players to starting positions
        for team in teams.values():
            if team.team_side == 'home':
                if team.strategy_type == 'defensive':
                    positions = {
                        'goalkeeper': np.array([-4.3, 0.0]),
                        'defender1': np.array([-3.0, -1.0]),
                        'defender2': np.array([-3.0, 1.0]),
                        'striker': np.array([-0.8, 0.0])
                    }
                elif team.strategy_type == 'offensive':
                    positions = {
                        'goalkeeper': np.array([-4.3, 0.0]),
                        'defender': np.array([-3.0, 0.0]),
                        'striker1': np.array([-0.8, -1.0]),
                        'striker2': np.array([-0.8, 1.0])
                    }
                else:  # balanced
                    positions = {
                        'goalkeeper': np.array([-4.3, 0.0]),
                        'defender': np.array([-3.0, 0.0]),
                        'midfielder': np.array([-1.5, 1.0]),
                        'striker': np.array([-0.8, -1.0])
                    }
            else:  # away team
                if team.strategy_type == 'defensive':
                    positions = {
                        'goalkeeper': np.array([4.3, 0.0]),
                        'defender1': np.array([3.0, -1.0]),
                        'defender2': np.array([3.0, 1.0]),
                        'striker': np.array([0.8, 0.0])
                    }
                elif team.strategy_type == 'offensive':
                    positions = {
                        'goalkeeper': np.array([4.3, 0.0]),
                        'defender': np.array([3.0, 0.0]),
                        'striker1': np.array([0.8, -1.0]),
                        'striker2': np.array([0.8, 1.0])
                    }
                else:  # balanced
                    positions = {
                        'goalkeeper': np.array([4.3, 0.0]),
                        'defender': np.array([3.0, 0.0]),
                        'midfielder': np.array([1.5, -1.0]),
                        'striker': np.array([0.8, 1.0])
                    }
            
            for player in team.players:
                player.position = positions[player.role].copy()
                player.velocity = np.array([0.0, 0.0])
                player.has_ball = False
                if hasattr(player, 'fsm') and player.fsm:
                    player.fsm.change_state(PlayerState.POSITIONING)
            
            print("Game reset after goal!")

    def constrain_ball_to_pitch(self, ball, pitch):
        # Keep ball in bounds
        half_length = pitch.length / 2
        half_width = pitch.width / 2

        if ball.position[0] > half_length:
            ball.position[0] = half_length
            ball.velocity[0] = -ball.velocity[0] * ball.bounce_coefficient
        elif ball.position[0] < -half_length:
            ball.position[0] = -half_length
            ball.velocity[0] = -ball.velocity[0] * ball.bounce_coefficient

        if ball.position[1] > half_width:
            ball.position[1] = half_width
            ball.velocity[1] = -ball.velocity[1] * ball.bounce_coefficient
        elif ball.position[1] < -half_width:
            ball.position[1] = -half_width
            ball.velocity[1] = -ball.velocity[1] * ball.bounce_coefficient

    def check_player_ball_collision(self, player, teams, ball):
        """Check and handle collisions between players and the ball"""
        if ball.owner is player:  # Skip if player already owns ball
            return
        
        # Skip collision if ball was just passed and is from this player
        if hasattr(ball, 'last_owner') and ball.last_owner is player and hasattr(ball, 'pass_cooldown') and ball.pass_cooldown > 0:
            return
            
        # Calculate distance between player and ball
        player_pos = np.array(player.position)
        ball_pos = np.array(ball.position)
        distance = np.linalg.norm(player_pos - ball_pos)
        
        # Check if player is close enough to ball
        if distance < player.radius + ball.radius:
            # If ball is owned by opponent, attempt to steal
            if ball.owner is not None and ball.owner.team != player.team:
                if player.can_steal():
                    if player.attempt_steal(ball.owner):
                        # Successful steal
                        print(f"{player.role} stole the ball from {ball.owner.role}!")
                        ball.owner.has_ball = False
                        ball.owner = player
                        player.has_ball = True
                        player.ball = ball
                        ball.velocity = np.array([0.0, 0.0])
                        player.fsm.change_state(PlayerState.POSSESSION)
                        
                        # Update other players
                        for team in teams.values():
                            for other in team.players:
                                if other != player:
                                    other.has_ball = False
                                    if other.fsm:
                                        other.fsm.change_state(PlayerState.POSITIONING)
                return
                
            # Don't take ball if another player has it and we didn't steal
            if ball.owner is not None:
                return
                    
            print(f"{player.role} got the ball!")
            
            # Give ball to player
            ball.owner = player
            player.has_ball = True
            player.ball = ball
            
            # Stop ball movement
            ball.velocity = np.array([0.0, 0.0])
            
            # Update player state
            player.fsm.change_state(PlayerState.POSSESSION)
            
            # Update other players
            for team in teams.values():
                for other in team.players:
                    if other != player:
                        other.has_ball = False
                        if other.fsm:
                            other.fsm.change_state(PlayerState.POSITIONING)

    def render(self, pitch, teams, ball):
        # Always render the ball and players
        all_players = []
        for team in teams.values():
            all_players.extend(team.players)
        
        # Clear and redraw everything
        pitch.draw(self.screen, players=all_players, ball=ball)
        
        # Render score
        self.render_score()
        
        # Render goal message if just scored
        if self.goal_scored:
            self.render_goal_message()
            
        pygame.display.flip()

    def render_score(self):
        font = pygame.font.Font(None, 36)
        score_text = f"Score: Home {self.score['home']} - {self.score['away']} Away"
        text_surface = font.render(score_text, True, (255, 255, 255))
        self.screen.blit(text_surface, (self.screen.get_width()/2 - text_surface.get_width()/2, 10))

    def render_goal_message(self):
        """Display GOAL! message after scoring"""
        font = pygame.font.Font(None, 72)
        goal_text = "GOAL!"
        text_surface = font.render(goal_text, True, (255, 255, 0))  # Yellow color
        text_rect = text_surface.get_rect(center=(self.screen.get_width()/2, self.screen.get_height()/2))
        self.screen.blit(text_surface, text_rect)

    def render_debug_info(self, teams, ball):
        font = pygame.font.Font(None, 24)
        y = 40
        
        # Ball info
        ball_info = [
            f"Ball pos: ({ball.position[0]:.2f}, {ball.position[1]:.2f})",
            f"Ball vel: ({ball.velocity[0]:.2f}, {ball.velocity[1]:.2f})",
            f"Ball owner: {ball.owner.role if ball.owner else 'None'}"
        ]
        
        for info in ball_info:
            text = font.render(info, True, (255, 255, 255))
            self.screen.blit(text, (10, y))
            y += 20