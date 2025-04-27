import pygame
import numpy as np
from fsm import PlayerState
from tactical_engine import TacticalEngine
from physics_engine import PhysicsEngine
from game_states import GameState 
from enum import Enum
class GameState(Enum):
    KICKOFF = "kickoff"
    PLAY = "play"
    GOAL = "goal"
    RESET = "reset"

class GameEngine:
    def __init__(self):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.debug_mode = True
        self.score = {"home": 0, "away": 0}
        self.goal_width = 2.6
        self.reset_delay = 2.0
        self.reset_timer = 0
        self.kickoff_timer = 0
        self.kickoff_delay = 1.0  # Delay before kickoff is allowed
        self.goal_scored = False
        self.event_handlers = []  # List of additional event handlers
        
        # Game state management
        self.game_state = GameState.KICKOFF
        self.kickoff_team = "home"  # Initial kickoff team
        self.last_team_scored = None  # Track who scored last
        self.center_circle_radius = 0.75  # Center circle radius in meters
        self.kickoff_taken = False

    def run(self, pitch, teams, ball):
        self.screen = pitch.initialize_display()
        self.running = True
        
        # Initial kickoff setup
        self.setup_kickoff(teams, ball, self.kickoff_team)
        
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
        if self.game_state == GameState.GOAL:
            self.reset_timer += dt
            if self.reset_timer >= self.reset_delay:
                # After goal delay, set up kickoff
                self.game_state = GameState.KICKOFF
                
                # The team that did NOT score gets the kickoff
                self.kickoff_team = "away" if self.last_team_scored == "home" else "home"
                
                print(f"Goal by {self.last_team_scored} team, {self.kickoff_team} team gets kickoff")
                
                # Set up the kickoff
                self.setup_kickoff(teams, ball, self.kickoff_team)
                self.reset_timer = 0
                self.goal_scored = False
            return
            
        elif self.game_state == GameState.KICKOFF:
            # Update kickoff timer
            if not self.kickoff_taken:
                self.kickoff_timer += dt
                
                # Check if any player has moved the ball
                if ball.owner is not None or np.any(ball.velocity):
                    self.kickoff_taken = True
                    self.game_state = GameState.PLAY
                    print(f"Kickoff taken by {self.kickoff_team} team")
                elif self.kickoff_timer >= self.kickoff_delay:
                    # After kickoff delay, allow kickoff team to move the ball
                    pass
                
                # Enforce kickoff rules (opponents stay outside center circle)
                self.enforce_kickoff_rules(teams, ball)
            else:
                # If kickoff was taken but ball is still in center circle
                if np.linalg.norm(ball.position) < self.center_circle_radius:
                    # Still in kickoff phase
                    pass
                else:
                    # Ball has left center circle, kickoff is complete
                    self.game_state = GameState.PLAY
                    print("Kickoff complete, game in play")
                    
        ball.update(dt)
    
        # Important: Always update teams regardless of game state
        for team in teams.values():
            # Update each team's players
            team.update(dt, ball.position)
        
        # Check for goals before constraining ball
        goal_result = self.check_for_goal(ball)
        if goal_result:
            self.goal_scored = True
            self.last_team_scored = goal_result
            self.game_state = GameState.GOAL
            return
            
        self.constrain_ball_to_pitch(ball, pitch)
        
        # Check collisions for each player
        for team_side, team in teams.items():
            for player in team.players:
                # Set ball reference for each player
                player.ball = ball
                
                # Only kickoff team can touch ball during kickoff
                if self.game_state == GameState.KICKOFF and not self.kickoff_taken:
                    if team_side != self.kickoff_team:
                        continue
                        
                self.check_player_ball_collision(player, teams, ball)
    def setup_kickoff(self, teams, ball, kickoff_team):
        """Set up the game for kickoff"""
        # Reset ball
        ball.position = np.array([0.0, 0.0])
        ball.velocity = np.array([0.0, 0.0])
        ball.owner = None
        
        # Reset all players to kickoff positions
        # Kickoff team gets one player at center
        for team in teams.values():
            if team.team_side == 'home':
                if team.strategy_type == 'defensive':
                    positions = {
                        'goalkeeper': np.array([-4.3, 0.0]),
                        'defender1': np.array([-3.0, -1.0]),
                        'defender2': np.array([-3.0, 1.0]),
                        'striker': np.array([-1.0, 0.0] if kickoff_team == 'home' else [-1.2, 0.0])
                    }
                elif team.strategy_type == 'offensive':
                    positions = {
                        'goalkeeper': np.array([-4.3, 0.0]),
                        'defender': np.array([-3.0, 0.0]),
                        'striker1': np.array([-1.0, -1.0]),
                        'striker2': np.array([-1.0, 0.0] if kickoff_team == 'home' else [-1.2, 1.0])
                    }
                else:  # balanced
                    positions = {
                        'goalkeeper': np.array([-4.3, 0.0]),
                        'defender': np.array([-3.0, 0.0]),
                        'midfielder': np.array([-1.5, 1.0]),
                        'striker': np.array([-1.0, 0.0] if kickoff_team == 'home' else [-1.2, -1.0])
                    }
            else:  # away team
                if team.strategy_type == 'defensive':
                    positions = {
                        'goalkeeper': np.array([4.3, 0.0]),
                        'defender1': np.array([3.0, -1.0]),
                        'defender2': np.array([3.0, 1.0]),
                        'striker': np.array([1.0, 0.0] if kickoff_team == 'away' else [1.2, 0.0])
                    }
                elif team.strategy_type == 'offensive':
                    positions = {
                        'goalkeeper': np.array([4.3, 0.0]),
                        'defender': np.array([3.0, 0.0]),
                        'striker1': np.array([1.0, -1.0]),
                        'striker2': np.array([1.0, 0.0] if kickoff_team == 'away' else [1.2, 1.0])
                    }
                else:  # balanced
                    positions = {
                        'goalkeeper': np.array([4.3, 0.0]),
                        'defender': np.array([3.0, 0.0]),
                        'midfielder': np.array([1.5, -1.0]),
                        'striker': np.array([1.0, 0.0] if kickoff_team == 'away' else [1.2, 1.0])
                    }
                    
            # Place the kickoff striker at center for the kicking team
            if team.team_side == kickoff_team:
                # Find the striker
                for player in team.players:
                    if player.role == 'striker' or player.role.startswith('striker'):
                        if team.team_side == 'home':
                            positions[player.role] = np.array([-0.2, 0.0])  # Slightly behind center
                        else:
                            positions[player.role] = np.array([0.2, 0.0])  # Slightly behind center
                        break
            
            # Apply positions to all players
            for player in team.players:
                player.position = positions[player.role].copy()
                player.velocity = np.array([0.0, 0.0])
                player.has_ball = False
                player.target_position = positions[player.role].copy()
                if hasattr(player, 'fsm') and player.fsm:
                    player.fsm.change_state(PlayerState.POSITIONING)
        
        # Reset kickoff state
        self.kickoff_taken = False
        self.kickoff_timer = 0.0
        print(f"Kickoff set up for {kickoff_team} team")

        # Explicitly update team strategies to set new target positions
        for team_side, team in teams.items():
            team.strategy.update_player_positions(team.players, ball.position)
            
            for player in team.players:
                print(f"{team_side} {player.role} target position: {player.target_position}")

    def enforce_kickoff_rules(self, teams, ball):
        """Enforce kickoff rules: opponents stay outside center circle until ball is touched"""
        for team in teams.values():
            # Skip the kickoff team
            if team.team_side == self.kickoff_team:
                continue
                
            # Keep opponents outside center circle
            for player in team.players:
                distance_to_center = np.linalg.norm(player.position)
                if distance_to_center < self.center_circle_radius:
                    # Calculate direction away from center
                    if distance_to_center > 0:
                        direction = player.position / distance_to_center
                    else:
                        # If at center, move in team direction
                        direction = np.array([1.0, 0.0]) if team.team_side == 'away' else np.array([-1.0, 0.0])
                        
                    # Move player to edge of center circle
                    player.position = direction * self.center_circle_radius
                    
                    # Stop player's movement
                    player.velocity = np.array([0.0, 0.0])

    def check_for_goal(self, ball):
        # Check home goal (left side)
        if ball.position[0] < -4.5 and abs(ball.position[1]) < self.goal_width/2:
            self.score["away"] += 1
            print(f"GOAL for Away team! Score: Home {self.score['home']} - {self.score['away']} Away")
            return "away"  # Team that scored
            
        # Check away goal (right side)
        if ball.position[0] > 4.5 and abs(ball.position[1]) < self.goal_width/2:
            self.score["home"] += 1
            print(f"GOAL for Home team! Score: Home {self.score['home']} - {self.score['away']} Away")
            return "home"  # Team that scored
            
        return None

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
        
        # Render game state messages
        if self.game_state == GameState.KICKOFF and not self.kickoff_taken:
            self.render_kickoff_message()
        elif self.game_state == GameState.GOAL:
            self.render_goal_message()
            
        # Draw center circle during kickoff
        if self.game_state == GameState.KICKOFF:
            self.render_center_circle(pitch)
            
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

    def render_kickoff_message(self):
        """Display kickoff message"""
        font = pygame.font.Font(None, 36)
        kickoff_text = f"{self.kickoff_team.upper()} TEAM KICKOFF"
        text_surface = font.render(kickoff_text, True, (255, 255, 255))
        text_rect = text_surface.get_rect(center=(self.screen.get_width()/2, self.screen.get_height() - 30))
        self.screen.blit(text_surface, text_rect)
        
        # Add countdown if in delay period
        if self.kickoff_timer < self.kickoff_delay:
            countdown = f"Kickoff in: {(self.kickoff_delay - self.kickoff_timer):.1f}s"
            count_surface = font.render(countdown, True, (255, 255, 255))
            count_rect = count_surface.get_rect(center=(self.screen.get_width()/2, self.screen.get_height() - 60))
            self.screen.blit(count_surface, count_rect)

    def render_center_circle(self, pitch):
        """Highlight the center circle during kickoff"""
        center_x, center_y = pitch.to_screen_coords(0, 0)
        radius = int(self.center_circle_radius * pitch.SCALE)
        
        # Draw transparent circle (indicating kickoff zone)
        s = pygame.Surface((radius*2, radius*2), pygame.SRCALPHA)
        pygame.draw.circle(s, (255, 255, 255, 30), (radius, radius), radius)
        self.screen.blit(s, (center_x-radius, center_y-radius))
        
        # Draw circle outline
        pygame.draw.circle(self.screen, (255, 255, 255), (center_x, center_y), radius, 2)

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