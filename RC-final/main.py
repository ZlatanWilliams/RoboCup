import pygame
import sys
import numpy as np
from pitch import Pitch
from players import Team
from ball import Ball
from game_engine import GameEngine
from metric import MatchStatsTracker, modify_game_engine, finalize_match_stats

def initialize_game():
    """Initialize all game components"""
    # Initialize pygame
    pygame.init()
    
    # Create game components
    pitch = Pitch()
    ball = Ball()
    
    # Create teams with different strategies
    away_team = Team('away', strategy_type='offensive')  # Away team plays defensive
    home_team = Team('home', strategy_type='defensive')  # Home team plays offensive
    
    # Create engines
    game_engine = GameEngine()
    
    # Create match stats tracker with game engine reference
    match_stats_tracker = MatchStatsTracker(home_team, away_team, game_engine)
    
    # Modify game engine to integrate match stats tracking
    modify_game_engine(game_engine, match_stats_tracker)
    
    # Set up initial ball position
    ball.position[0] = 0  # Center X
    ball.position[1] = 0  # Center Y
    
    # Set up team relationships
    for home_player in home_team.players:
        home_player.ball = ball
        # Set teammates
        home_player.teammates = [p for p in home_team.players if p != home_player]
        # Set opponents
        home_player.opponents = away_team.players

    for away_player in away_team.players:
        away_player.ball = ball
        # Set teammates
        away_player.teammates = [p for p in away_team.players if p != away_player]
        # Set opponents
        away_player.opponents = home_team.players
    
    return pitch, home_team, away_team, ball, game_engine, match_stats_tracker

def main():
    try:
        # Initialize all components
        pitch, home_team, away_team, ball, game_engine, match_stats_tracker = initialize_game()
        
        # Create teams dictionary
        teams = {
            "home": home_team,
            "away": away_team
        }
        
        # Print team strategies for confirmation
        print(f"Home team strategy: {home_team.strategy_type}")
        print(f"Away team strategy: {away_team.strategy_type}")
        
        # Set up display
        screen = pitch.initialize_display()
        pygame.display.set_caption("Robot Soccer Simulation")
        
        # Main game loop handled by game engine
        game_engine.run(pitch, teams, ball)
        
        # Finalize and export match stats
        finalize_match_stats(match_stats_tracker)
        
    except Exception as e:
        print(f"Error in main game loop: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    main()