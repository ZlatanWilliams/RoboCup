import pygame
import numpy as np
from pitch import Pitch
from players import Team
from ball import Ball
from game_engine import GameEngine

def main():
    pygame.init()
    clock = pygame.time.Clock()
    
    pitch = Pitch()
    screen = pitch.initialize_display()
    
    # ✅ Instead of creating players manually, use `Team` class
    home_team = Team('home')  # Initializes all home players
    away_team = Team('away')  # Initializes all away players

    ball = Ball()
    engine = GameEngine()
    
    # ✅ Pass the teams directly to the game engine
    engine.run(pitch, {"home": home_team.players, "away": away_team.players}, ball)

if __name__ == "__main__":
    main()
