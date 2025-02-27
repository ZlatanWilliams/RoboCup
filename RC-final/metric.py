import csv
import numpy as np
import os
from typing import Dict

class MatchStatsTracker:
    def __init__(self, home_team, away_team, game_engine):
        """
        Initialize match statistics tracker.
        """
        self.home_team = home_team
        self.away_team = away_team
        self.game_engine = game_engine
        
        # Total match time
        self.total_match_time = 0.0
        
        # Shots on target (only striker shots that result in a goal or are intercepted)
        self.shots = {
            'home': [],
            'away': []
        }
        self.possession_time = {'home': 0.0, 'away': 0.0}
        # Track the last striker who touched the ball
        self.last_striker = None

    def update(self, ball, dt):
        """
        Update match statistics for each frame.
        """
        if ball.owner:
            team_side = ball.owner.team.team_side
            self.possession_time[team_side] += dt
        self.total_match_time += dt

    def track_shot_result(self, striker, result_type):
        """
        Record the result of a striker's shot on target.
        
        :param striker: Striker who took the shot.
        :param result_type: 'goal' or 'intercepted'.
        """
        team_side = striker.team.team_side
        shot_data = {
            'player': striker.role,
            'position': striker.position.tolist(),
            'result': result_type,
            'timestamp': self.total_match_time
        }
        self.shots[team_side].append(shot_data)
        print(f"[DEBUG] Shot recorded for {team_side}: {shot_data}")

    def calculate_possession_percentage(self) -> Dict[str, float]:
        """
        Calculate possession percentage for each team
        
        :return: Dictionary with possession percentages
        """
        possession_percentages = {}
        for team_side, time in self.possession_time.items():
            percentage = (time / self.total_match_time * 100) if self.total_match_time > 0 else 0
            possession_percentages[team_side] = round(percentage, 2)
        return possession_percentages
    
    def export_match_stats(self, filename='match_stats.csv'):
        file_exists = os.path.isfile(filename)
        with open(filename, 'a', newline='') as csvfile:
            fieldnames = ['metric', 'home_value', 'away_value']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists or os.path.getsize(filename) == 0:
                writer.writeheader()
            
            # Convert team strategy objects to strings.
            home_strategy = self.home_team.strategy
            away_strategy = self.away_team.strategy
            if not isinstance(home_strategy, str):
                home_strategy = home_strategy.__class__.__name__
            if not isinstance(away_strategy, str):
                away_strategy = away_strategy.__class__.__name__
            
            writer.writerow({
                'metric': 'Team Strategy',
                'home_value': home_strategy,
                'away_value': away_strategy
            })
            
            poss_percentages = self.calculate_possession_percentage()
            writer.writerow({
                'metric': 'Possession (%)',
                'home_value': poss_percentages.get('home', 0),
                'away_value': poss_percentages.get('away', 0)
            })
            
            writer.writerow({
                'metric': 'Total Shots On Target',
                'home_value': len(self.shots['home']),
                'away_value': len(self.shots['away'])
            })
            
            writer.writerow({
                'metric': 'Goals',
                'home_value': self.game_engine.score['home'],
                'away_value': self.game_engine.score['away']
            })
        
        self._export_shot_details()



    def _export_shot_details(self, filename='shot_details.csv'):
        file_exists = os.path.isfile(filename)
        with open(filename, 'a', newline='') as csvfile:
            fieldnames = ['team', 'player', 'x_position', 'y_position', 'result', 'timestamp']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists or os.path.getsize(filename) == 0:
                writer.writeheader()
            for team_side in ['home', 'away']:
                for shot in self.shots[team_side]:
                    writer.writerow({
                        'team': team_side,
                        'player': shot['player'],
                        'x_position': shot['position'][0],
                        'y_position': shot['position'][1],
                        'result': shot['result'],
                        'timestamp': shot['timestamp']
                    })


def modify_game_engine(game_engine, match_stats_tracker):
    """
    Modify the game engine to integrate match stats tracking for striker shots on target.
    """
    original_check_for_goal = game_engine.check_for_goal
    original_update = game_engine.update
    original_check_player_ball_collision = game_engine.check_player_ball_collision
    
    def modified_check_for_goal(ball):
        # Call the original goal check first.
        goal_result = original_check_for_goal(ball)
        if goal_result:
            # If a goal is scored, assume the last striker attempted a shot on target.
            if match_stats_tracker.last_striker:
                match_stats_tracker.track_shot_result(match_stats_tracker.last_striker, 'goal')
                match_stats_tracker.last_striker = None
        return goal_result
    
    def modified_check_player_ball_collision(player, teams, ball):
        # Capture the original ball owner.
        original_owner = ball.owner
        
        # Call the original collision handling.
        original_check_player_ball_collision(player, teams, ball)
        
        # If a striker now gets the ball, record them as the last striker.
        if ball.owner and ball.owner.role == 'striker':
            match_stats_tracker.last_striker = ball.owner
        
        # If a goalkeeper now has the ball, assume a shot on target was taken by the last striker.
        if ball.owner and ball.owner.role == 'goalkeeper':
            if match_stats_tracker.last_striker:
                match_stats_tracker.track_shot_result(match_stats_tracker.last_striker, 'intercepted')
                match_stats_tracker.last_striker = None

    def modified_update(pitch, teams, ball, dt):
        match_stats_tracker.update(ball, dt)
        original_update(pitch, teams, ball, dt)
    
    game_engine.check_for_goal = modified_check_for_goal
    game_engine.update = modified_update
    game_engine.check_player_ball_collision = modified_check_player_ball_collision

def finalize_match_stats(match_stats_tracker):
    match_stats_tracker.export_match_stats()
    print("Match stats exported successfully!")
