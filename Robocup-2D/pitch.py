import pygame
import math

class Pitch:
    def __init__(self):
        # Main pitch dimensions in meters
        self.length = 9.0 
        self.width = 6.0 
        
        # Goal specifications
        self.goal_depth = 0.6
        self.goal_width = 2.6
        
        # Field markings dimensions
        self.goal_area_length = 1.0
        self.goal_area_width = 3.0
        self.penalty_mark_distance = 1.5
        self.center_circle_diameter = 1.5
        self.border_strip_width = 1.0
        self.penalty_area_length = 2.0
        self.penalty_area_width = 3.0
        
        self.center_circle_radius = self.center_circle_diameter / 2
        self.LINE_THICKNESS = 2
        
        # Colors
        self.GRASS_COLOR = (44, 92, 52)
        self.LINE_COLOR = (255, 255, 255)
        self.GOAL_COLOR = (102, 102, 102)

        # Screen dimensions and scaling
        self.SCREEN_WIDTH = 1280
        self.SCREEN_HEIGHT = 720
        
        # Calculate scaling (with margin)
        width_scale = (self.SCREEN_WIDTH - 100) / self.length
        height_scale = (self.SCREEN_HEIGHT - 100) / self.width
        self.SCALE = min(width_scale, height_scale)
        
        # Offsets for centering the pitch on the screen
        self.offset_x = (self.SCREEN_WIDTH - (self.length * self.SCALE)) / 2
        self.offset_y = (self.SCREEN_HEIGHT - (self.width * self.SCALE)) / 2

    def initialize_display(self):
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.display.set_caption("NAO Robot Football Pitch")
        return self.screen

    def to_screen_coords(self, x, y):
        """Convert field coordinates (meters) to screen coordinates (pixels)."""
        screen_x = self.offset_x + (x + self.length/2) * self.SCALE
        screen_y = self.offset_y + (self.width/2 - y) * self.SCALE
        return int(screen_x), int(screen_y)

    def draw(self, screen, players=None, ball=None):
        screen.fill(self.GRASS_COLOR)
        self._draw_main_pitch(screen)
        self._draw_field_markings(screen)
        self._draw_goals(screen)
        if ball:
            self._draw_ball(screen, ball)
        if players:
            self._draw_players(screen, players)

    def _draw_main_pitch(self, screen):
        start_x, start_y = self.to_screen_coords(-self.length/2, self.width/2)
        end_x, end_y = self.to_screen_coords(self.length/2, -self.width/2)
        pygame.draw.rect(screen, self.LINE_COLOR,
                         (start_x, start_y, end_x - start_x, end_y - start_y),
                         self.LINE_THICKNESS)

    def _draw_goals(self, screen):
        # Left goal
        left_x, left_y = self.to_screen_coords(-self.length/2 - self.goal_depth, self.goal_width/2)
        goal_width_pixels = self.goal_depth * self.SCALE
        goal_height_pixels = self.goal_width * self.SCALE
        pygame.draw.rect(screen, self.GOAL_COLOR,
                         (left_x, left_y, goal_width_pixels, goal_height_pixels))
        # Right goal
        right_x, right_y = self.to_screen_coords(self.length/2, self.goal_width/2)
        pygame.draw.rect(screen, self.GOAL_COLOR,
                         (right_x, right_y, goal_width_pixels, goal_height_pixels))

    def _draw_field_markings(self, screen):
        # Center line
        start_x, start_y = self.to_screen_coords(0, self.width/2)
        end_x, end_y = self.to_screen_coords(0, -self.width/2)
        pygame.draw.line(screen, self.LINE_COLOR, (start_x, start_y), (end_x, end_y), self.LINE_THICKNESS)
        
        # Center circle and dot
        center_x, center_y = self.to_screen_coords(0, 0)
        radius = int(self.center_circle_radius * self.SCALE)
        pygame.draw.circle(screen, self.LINE_COLOR, (center_x, center_y), radius, self.LINE_THICKNESS)
        pygame.draw.circle(screen, self.LINE_COLOR, (center_x, center_y), 3)
        
        # Goal and penalty areas
        for side in [-1, 1]:
            x = side * self.length/2
            x_offset = 0 if side == -1 else -self.goal_area_length
            start_x, start_y = self.to_screen_coords(x + x_offset, self.goal_area_width/2)
            rect_width = self.goal_area_length * self.SCALE
            rect_height = self.goal_area_width * self.SCALE
            pygame.draw.rect(screen, self.LINE_COLOR,
                             (start_x, start_y, rect_width, rect_height), self.LINE_THICKNESS)
            
            x_offset = 0 if side == -1 else -self.penalty_area_length
            start_x, start_y = self.to_screen_coords(x + x_offset, self.penalty_area_width/2)
            rect_width = self.penalty_area_length * self.SCALE
            rect_height = self.penalty_area_width * self.SCALE
            pygame.draw.rect(screen, self.LINE_COLOR,
                             (start_x, start_y, rect_width, rect_height), self.LINE_THICKNESS)
        
        # Penalty marks
        for side in [-1, 1]:
            x = side * (self.length/2 - self.penalty_mark_distance)
            mark_x, mark_y = self.to_screen_coords(x, 0)
            pygame.draw.circle(screen, self.LINE_COLOR, (mark_x, mark_y), 3)

    def _draw_ball(self, screen, ball):
        ball_x, ball_y = self.to_screen_coords(ball.position[0], ball.position[1])
        pygame.draw.circle(screen, (255, 255, 0),
                           (ball_x, ball_y), int(ball.radius * self.SCALE))

    def _draw_players(self, screen, players):
        for player in players:
            props = player.get_visual_properties() if hasattr(player, "get_visual_properties") else {}
            if props:
                pos_x, pos_y = self.to_screen_coords(props.get('position', (0,0))[0],
                                                      props.get('position', (0,0))[1])
                pygame.draw.circle(screen, props.get('color', (255,255,255)),
                                   (pos_x, pos_y), int(props.get('size', 0.15) * self.SCALE))
                font = pygame.font.Font(None, 24)
                text = font.render(props.get('label', ''), True, self.LINE_COLOR)
                text_rect = text.get_rect(center=(pos_x, pos_y - 20))
                screen.blit(text, text_rect)
