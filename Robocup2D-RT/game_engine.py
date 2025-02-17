import pygame
import math
from ball import Ball
from players import Player
from fsm import PlayerFSM

class GameEngine:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((1280, 720))
        pygame.display.set_caption("RoboCup 2D Soccer Simulation")
        self.clock = pygame.time.Clock()
        self.running = False

        self.scale_x = 1280 / (9.0 + 2.0)
        self.scale_y = 720 / (6.0 + 2.0)

    def run(self, pitch, teams, ball):
        for team in teams.values():
            for player in team:
                player.ball = ball
        self.running = True
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            self._handle_events()
            self._update(pitch, teams, ball, dt)
            self._render(pitch, teams, ball)
        pygame.quit()

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

    def _update(self, pitch, teams, ball, dt):
        ball.update(dt, pitch.width, pitch.length)

        for team in teams.values():
            for player in team:
                player.update(dt)
                player.fsm.update()
                self._check_player_ball_collision(player, teams, ball)

    def _check_player_ball_collision(self, player, teams, ball):
        distance = math.hypot(player.position[0] - ball.position[0],
                            player.position[1] - ball.position[1])

        if distance < player.radius + ball.radius:
            if ball.owner is None or ball.owner != player:  
                ball.owner = player
                player.has_ball = True
                ball.velocity = (0, 0)
                
                # Notify FSM that the player has the ball
                if player.fsm:
                    player.fsm.change_state("possession")

            # Ensure other players recognize they don’t have possession
            for team in teams.values():
                for other_player in team:
                    if other_player != player:
                        other_player.has_ball = False
            
        

    def _render(self, pitch, teams, ball):
        pitch.draw(self.screen)  # ✅ Use the full pitch rendering

        for team in teams.values():
            for player in team:
                pos = self._scale_position(player.position, pitch)
                color = (0, 0, 255) if player.team == 'home' else (255, 0, 0)
                pygame.draw.circle(self.screen, color, pos, 20)
                font = pygame.font.Font(None, 24)
                text = font.render(player.role[0].upper(), True, (255, 255, 255))
                self.screen.blit(text, (pos[0] - 8, pos[1] - 10))
        ball_pos = self._scale_position(ball.position, pitch)
        pygame.draw.circle(self.screen, (139, 69, 19), ball_pos, int(ball.radius * self.scale_x))
        pygame.display.flip()

    def _scale_position(self, pos, pitch):
        """Converts field coordinates (meters) to screen coordinates (pixels)."""
        return pitch.to_screen_coords(pos[0], pos[1])  # ✅ Use `Pitch` method


    def _draw_pitch(self, pitch):
        pygame.draw.line(self.screen, (255, 255, 255),
                         self._scale_position((0, -3)),
                         self._scale_position((0, 3)), 2) 
        center = self._scale_position((0, 0))
        pygame.draw.circle(self.screen, (255, 255, 255), center, int(0.75 * self.scale_x), 2)
