from enum import Enum

class GameState(Enum):
    """Game states for managing kickoff, regular play, and goals"""
    KICKOFF = "kickoff"
    PLAY = "play"
    GOAL = "goal"
    RESET = "reset"