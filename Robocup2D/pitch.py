import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

class Pitch:
    def __init__(self):
        # Main pitch dimensions
        self.length = 9.0       # Total length in meters
        self.width = 6.0        # Total width in meters
        
        # Goal specifications
        self.goal_depth = 0.6
        self.goal_width = 2.6
        self.goal_height = 1.2  # Not used in 2D visualization
        
        # Field markings
        self.goal_area_length = 1.0
        self.goal_area_width = 3.0
        self.penalty_mark_distance = 1.5
        self.center_circle_diameter = 1.5
        self.border_strip_width = 1.0
        self.penalty_area_length = 2.0
        self.penalty_area_width = 3.0  # Assumed value
        
        # Calculated properties
        self.center_circle_radius = self.center_circle_diameter / 2
        
    def draw(self, ax, players, ball):
        """Visualize the pitch with optional players"""
        fig, ax = plt.subplots(figsize=(10, 7))
        ax.set_aspect('equal')
        ax.set_title("NAO Robot Football Pitch")
        
        # Draw pitch elements
        self._draw_border(ax)
        self._draw_main_pitch(ax)
        self._draw_goals(ax)
        self._draw_field_markings(ax)

        self._draw_ball(ax, ball)

        
        # Draw players if provided
        if players:
            self._draw_players(ax, players)

        ax.set_xlim(-self.length/2 - 1, self.length/2 + 1)
        ax.set_ylim(-self.width/2 - 1, self.width/2 + 1)
        plt.axis('off')
        plt.show()

    def _draw_border(self, ax):
        """Draw outer border strip"""
        border_length = self.length + 2 * self.border_strip_width
        border_width = self.width + 2 * self.border_strip_width
        border_rect = patches.Rectangle(
            (-border_length/2, -border_width/2),
            border_length,
            border_width,
            linewidth=2,
            edgecolor='black',
            facecolor='#2c5c34'
        )
        ax.add_patch(border_rect)

    def _draw_ball(self, ax, ball):
        """Detailed leather ball with panel pattern"""
        # Main ball body
        ball_circle = patches.Circle(
            ball.position,
            radius=ball.radius,
            facecolor=ball.color,
            edgecolor=ball.panel_color,
            linewidth=1.5,
            zorder=20
        )
        ax.add_patch(ball_circle)
        
        # Add panel pattern
        for angle in range(0, 360, 360//ball.panels):
            rad = math.radians(angle)
            end_point = (
                ball.position[0] + ball.radius * math.cos(rad),
                ball.position[1] + ball.radius * math.sin(rad)
            )
            ax.plot(
                [ball.position[0], end_point[0]],
                [ball.position[1], end_point[1]],
                color=ball.panel_color,
                linewidth=1,
                alpha=0.7
            )

    def _draw_players(self, ax, players):
        """Draw all players on the pitch"""
        for player in players:
            props = player.get_visual_properties()
            ax.scatter(
                props['position'][0], 
                props['position'][1],
                s=props['size'],
                c=props['color'],
                edgecolors='white',
                linewidths=1.5,
                marker='o',
                zorder=10
            )
            # Add role label
            ax.text(
                props['position'][0], 
                props['position'][1] + 0.3,
                props['label'],
                color='white',
                ha='center', 
                va='center',
                fontsize=8,
                weight='bold'
            )
        
    def _draw_main_pitch(self, ax):
        """Draw the main playing surface"""
        pitch = patches.Rectangle(
            (-self.length/2, -self.width/2),
            self.length, self.width,
            linewidth=2, edgecolor='white', facecolor='#2c5c34'
        )
        ax.add_patch(pitch)
        
    def _draw_goals(self, ax):
        """Draw both goals"""
        # Right goal
        goal_right = patches.Rectangle(
            (self.length/2, -self.goal_width/2),
            self.goal_depth, self.goal_width,
            linewidth=2, edgecolor='white', facecolor='#666666'
        )
        ax.add_patch(goal_right)
        
        # Left goal
        goal_left = patches.Rectangle(
            (-self.length/2 - self.goal_depth, -self.goal_width/2),
            self.goal_depth, self.goal_width,
            linewidth=2, edgecolor='white', facecolor='#666666'
        )
        ax.add_patch(goal_left)
        
    def _draw_field_markings(self, ax):
        """Draw all field markings"""
        self._draw_center_circle(ax)
        self._draw_goal_areas(ax)
        self._draw_penalty_areas(ax)
        self._draw_penalty_marks(ax)
        self._draw_halfway_line(ax)
        
    def _draw_center_circle(self, ax):
        """Draw center circle and center spot"""
        center_circle = patches.Circle(
            (0, 0), self.center_circle_radius,
            linewidth=2, edgecolor='white', facecolor='none'
        )
        ax.add_patch(center_circle)
        ax.plot(0, 0, 'wo', markersize=5)
        
    def _draw_goal_areas(self, ax):
        """Draw goal areas at both ends"""
        # Right goal area
        goal_area_right = patches.Rectangle(
            (self.length/2 - self.goal_area_length, -self.goal_area_width/2),
            self.goal_area_length, self.goal_area_width,
            linewidth=2, edgecolor='white', facecolor='none'
        )
        ax.add_patch(goal_area_right)
        
        # Left goal area
        goal_area_left = patches.Rectangle(
            (-self.length/2, -self.goal_area_width/2),
            self.goal_area_length, self.goal_area_width,
            linewidth=2, edgecolor='white', facecolor='none'
        )
        ax.add_patch(goal_area_left)
        
    def _draw_penalty_areas(self, ax):
        """Draw penalty areas at both ends"""
        # Right penalty area
        penalty_area_right = patches.Rectangle(
            (self.length/2 - self.penalty_area_length, -self.penalty_area_width/2),
            self.penalty_area_length, self.penalty_area_width,
            linewidth=2, edgecolor='white', facecolor='none'
        )
        ax.add_patch(penalty_area_right)
        
        # Left penalty area
        penalty_area_left = patches.Rectangle(
            (-self.length/2, -self.penalty_area_width/2),
            self.penalty_area_length, self.penalty_area_width,
            linewidth=2, edgecolor='white', facecolor='none'
        )
        ax.add_patch(penalty_area_left)
        
    def _draw_penalty_marks(self, ax):
        """Draw penalty marks"""
        # Right penalty mark
        right_mark_x = self.length/2 - self.penalty_mark_distance
        ax.plot(right_mark_x, 0, 'wo', markersize=5)
        
        # Left penalty mark
        left_mark_x = -self.length/2 + self.penalty_mark_distance
        ax.plot(left_mark_x, 0, 'wo', markersize=5)
        
    def _draw_halfway_line(self, ax):
        """Draw halfway line"""
        ax.plot([0, 0], [self.width/2, -self.width/2], 'w-', linewidth=2)