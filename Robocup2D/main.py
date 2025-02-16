from pitch import Pitch
from players import Player
from game_objects import Ball, Goal
from game_engine import GameEngine

def initialize_players():
    """Create players with initial positions"""
    players = []
    
    # Left team (blue)
    players.append(Player('left', 'goalkeeper', (-4.2, 0)))
    players.append(Player('left', 'defender', (-3.0, 0)))
    players.append(Player('left', 'midfielder', (-1.5, 0)))
    players.append(Player('left', 'striker', (-0.5, 0)))
    
    # Right team (red)
    players.append(Player('right', 'goalkeeper', (4.2, 0)))
    players.append(Player('right', 'defender', (3.0, 0)))
    players.append(Player('right', 'midfielder', (1.5, 0)))
    players.append(Player('right', 'striker', (0.5, 0)))
    
    return players

def main():
    # Initialize game components
    pitch = Pitch()
    players = initialize_players()
    ball = Ball()
    
    # Give initial kick (example)
    ball.velocity = (3, 2)  # 3 m/s right, 2 m/s up 
    
    # Create and start game engine
    engine = GameEngine(pitch, players, ball)
    engine.start()

if __name__ == "__main__":
    main()