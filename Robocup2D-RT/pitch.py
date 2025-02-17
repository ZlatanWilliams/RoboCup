import pygame
import math

class Pitch:
    def __init__(self):
        # all main pitch dimensions in meters
        self.length = 9.0 
        self.width = 6.0 
        
        # Goal specifications
        self.goal_depth = 0.6
        self.goal_width = 2.6
        
        # Field markings
        self.goal_area_length = 1.0
        self.goal_area_width = 3.0
        self.penalty_mark_distance = 1.5
        self.center_circle_diameter = 1.5
        self.border_strip_width = 1.0
        self.penalty_area_length = 2.0
        self.penalty_area_width = 3.0
        
        # Calculated properties
        self.center_circle_radius = self.center_circle_diameter / 2
        
        # Display settings
        self.SCALE = 80  # pixels per meter
        self.LINE_THICKNESS = 2
        
        # Colors
        self.GRASS_COLOR = (44, 92, 52)    # Dark green
        self.LINE_COLOR = (255, 255, 255)  # White
        self.GOAL_COLOR = (102, 102, 102)  # Gray

        # Calculate window dimensions including border strips
        self.total_width = (self.width + 2 * self.border_strip_width) * self.SCALE
        self.total_length = (self.length + 2 * self.border_strip_width) * self.SCALE

    def initialize_display(self):
        self.screen = pygame.display.set_mode((int(self.total_length), int(self.total_width)))
        pygame.display.set_caption("NAO Robot Football Pitch")
        return self.screen

    def to_screen_coords(self, x, y):
        screen_x = (x + self.length/2 + self.border_strip_width) * self.SCALE
        screen_y = (y + self.width/2 + self.border_strip_width) * self.SCALE
        return int(screen_x), int(screen_y)

    def draw(self, screen, players=None, ball=None):
        screen.fill(self.GRASS_COLOR)
        
        self._draw_border(screen)
        self._draw_main_pitch(screen)
        self._draw_goals(screen)
        self._draw_field_markings(screen)
        
        if ball:
            self._draw_ball(screen, ball)
            
        if players:
            self._draw_players(screen, players)

    def _draw_border(self, screen):
        pygame.draw.rect(screen, self.GRASS_COLOR, (0, 0, self.total_length, self.total_width))
        
    def _draw_main_pitch(self, screen):
        start_x, start_y = self.to_screen_coords(-self.length/2, -self.width/2)
        rect_width = self.length * self.SCALE
        rect_height = self.width * self.SCALE
        pygame.draw.rect(screen, self.LINE_COLOR, 
                        (start_x, start_y, rect_width, rect_height), 
                        self.LINE_THICKNESS)

    def _draw_goals(self, screen):

        right_x, right_y = self.to_screen_coords(self.length/2, -self.goal_width/2)
        pygame.draw.rect(screen, self.GOAL_COLOR,
                        (right_x, right_y,
                         self.goal_depth * self.SCALE,
                         self.goal_width * self.SCALE))
        

        left_x, left_y = self.to_screen_coords(-self.length/2 - self.goal_depth, -self.goal_width/2)
        pygame.draw.rect(screen, self.GOAL_COLOR,
                        (left_x, left_y,
                         self.goal_depth * self.SCALE,
                         self.goal_width * self.SCALE))

    def _draw_field_markings(self, screen):

        self._draw_center_circle(screen)
        self._draw_goal_areas(screen)
        self._draw_penalty_areas(screen)
        self._draw_penalty_marks(screen)
        self._draw_halfway_line(screen)

    def _draw_center_circle(self, screen):

        center_x, center_y = self.to_screen_coords(0, 0)
        radius = int(self.center_circle_radius * self.SCALE)
        pygame.draw.circle(screen, self.LINE_COLOR, (center_x, center_y), radius, self.LINE_THICKNESS)
        pygame.draw.circle(screen, self.LINE_COLOR, (center_x, center_y), 3)

    def _draw_goal_areas(self, screen):

        right_x, right_y = self.to_screen_coords(self.length/2 - self.goal_area_length, -self.goal_area_width/2)
        pygame.draw.rect(screen, self.LINE_COLOR,
                        (right_x, right_y,
                         self.goal_area_length * self.SCALE,
                         self.goal_area_width * self.SCALE),
                        self.LINE_THICKNESS)
        

        left_x, left_y = self.to_screen_coords(-self.length/2, -self.goal_area_width/2)
        pygame.draw.rect(screen, self.LINE_COLOR,
                        (left_x, left_y,
                         self.goal_area_length * self.SCALE,
                         self.goal_area_width * self.SCALE),
                        self.LINE_THICKNESS)

    def _draw_penalty_areas(self, screen):

        right_x, right_y = self.to_screen_coords(self.length/2 - self.penalty_area_length, -self.penalty_area_width/2)
        pygame.draw.rect(screen, self.LINE_COLOR,
                        (right_x, right_y,
                         self.penalty_area_length * self.SCALE,
                         self.penalty_area_width * self.SCALE),
                        self.LINE_THICKNESS)
        

        left_x, left_y = self.to_screen_coords(-self.length/2, -self.penalty_area_width/2)
        pygame.draw.rect(screen, self.LINE_COLOR,
                        (left_x, left_y,
                         self.penalty_area_length * self.SCALE,
                         self.penalty_area_width * self.SCALE),
                        self.LINE_THICKNESS)

    def _draw_penalty_marks(self, screen):

        right_x, right_y = self.to_screen_coords(self.length/2 - self.penalty_mark_distance, 0)
        pygame.draw.circle(screen, self.LINE_COLOR, (right_x, right_y), 3)
        

        left_x, left_y = self.to_screen_coords(-self.length/2 + self.penalty_mark_distance, 0)
        pygame.draw.circle(screen, self.LINE_COLOR, (left_x, left_y), 3)

    def _draw_halfway_line(self, screen):

        top_x, top_y = self.to_screen_coords(0, -self.width/2)
        bottom_x, bottom_y = self.to_screen_coords(0, self.width/2)
        pygame.draw.line(screen, self.LINE_COLOR, 
                        (top_x, top_y), 
                        (bottom_x, bottom_y), 
                        self.LINE_THICKNESS)

    def _draw_ball(self, screen, ball):

        ball_x, ball_y = self.to_screen_coords(ball.position[0], ball.position[1])
        pygame.draw.circle(screen, ball.color, (ball_x, ball_y), 
                         int(ball.radius * self.SCALE))

    def _draw_players(self, screen, players):

        for player in players:
            props = player.get_visual_properties()
            pos_x, pos_y = self.to_screen_coords(props['position'][0], props['position'][1])
            

            pygame.draw.circle(screen, props['color'], 
                             (pos_x, pos_y), 
                             int(props['size'] * self.SCALE))
            

            font = pygame.font.Font(None, 24)
            text = font.render(props['label'], True, self.LINE_COLOR)
            text_rect = text.get_rect(center=(pos_x, pos_y - 20))
            screen.blit(text, text_rect)