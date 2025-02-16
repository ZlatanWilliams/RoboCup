import matplotlib.animation as animation
import matplotlib.pylab as plt

class GameEngine:
    def __init__(self, pitch, players, ball):
        self.pitch = pitch
        self.players = players
        self.ball = ball
        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.animation = None

    def _update_frame(self, frame):
        """Update function for animation"""
        # Update ball physics
        self.ball.update(dt=0.1)  # 100ms time step
        
        # Keep ball within pitch bounds
        self._enforce_boundaries()
        
        # Redraw everything
        self.pitch.draw(self.ax, self.players, self.ball)
        return []

    def _enforce_boundaries(self):
        """Realistic leather ball boundary collisions"""
        x, y = self.ball.position
        max_x = self.pitch.length/2 - self.ball.radius
        max_y = self.pitch.width/2 - self.ball.radius
        
        # X-axis boundaries with energy loss
        if abs(x) > max_x:
            self.ball.velocity = (
                -self.ball.velocity[0] * self.ball.bounce_coefficient,
                self.ball.velocity[1] * 0.9  # Reduce parallel component
            )
            x = max_x if x > 0 else -max_x
            
        # Y-axis boundaries
        if abs(y) > max_y:
            self.ball.velocity = (
                self.ball.velocity[0] * 0.9,
                -self.ball.velocity[1] * self.ball.bounce_coefficient
            )
            y = max_y if y > 0 else -max_y
            
        self.ball.position = (x, y)

    def start(self):
        """Start the game animation"""
        self.animation = animation.FuncAnimation(
            self.fig,
            self._update_frame,
            frames=100,
            interval=50,
            blit=False
        )
        plt.show()