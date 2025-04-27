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
        
        # Shots on target (only count goals and goalkeeper interceptions)
        self.shots = {
            'home': 0,
            'away': 0
        }
        
        # Shot details (for logging)
        self.shot_details = {
            'home': [],
            'away': []
        }
        
        # Possession tracking
        self.possession_time = {'home': 0.0, 'away': 0.0}
        self.last_recorded_ball_owner = None
        
        # Store previous scores to detect changes
        self.previous_score = {'home': 0, 'away': 0}
        
        print("[INIT] Match stats tracker initialized: tracking goals, goalkeeper interceptions, and possession")

    def update(self, ball, dt):
        """
        Update match statistics for each frame.
        """
        # Update possession tracking
        if ball.owner:
            team_side = ball.owner.team.team_side
            self.possession_time[team_side] += dt
            self.last_recorded_ball_owner = team_side
        elif self.last_recorded_ball_owner:
            # If no current owner but we had a previous owner, continue accumulating time
            # This handles cases where a player kicks the ball but it hasn't been picked up yet
            self.possession_time[self.last_recorded_ball_owner] += dt * 0.5  # Count at half rate when ball is free
            
        # Update total match time
        self.total_match_time += dt

    def record_shot(self, team_side, result_type, position=None):
        """
        Record a shot for the specified team.
        Only called for goals or goalkeeper interceptions.
        
        :param team_side: 'home' or 'away'
        :param result_type: 'goal' or 'intercepted'
        :param position: Optional shot position
        """
        # Increment shot counter
        self.shots[team_side] += 1
        
        # Create position data if not provided
        if position is None:
            position = [0, 0]
        
        # Create shot record for detailed export
        shot_data = {
            'player': 'striker',
            'position': position,
            'result': result_type,
            'timestamp': self.total_match_time
        }
        
        # Add to shot details
        self.shot_details[team_side].append(shot_data)
        
        print(f"[SHOT] {team_side} team: {result_type}. Total shots now: {self.shots[team_side]}")

    def calculate_possession_percentage(self) -> Dict[str, float]:
        """
        Calculate possession percentage for each team
        
        :return: Dictionary with possession percentages
        """
        total_possession_time = sum(self.possession_time.values())
        possession_percentages = {}
        
        # Calculate percentage for each team
        for team_side, time in self.possession_time.items():
            if total_possession_time > 0:
                percentage = (time / total_possession_time) * 100
            else:
                percentage = 50.0  # Default to 50% if no time recorded
                
            possession_percentages[team_side] = round(percentage, 2)
        
        # Verify percentages sum to 100%
        total_percentage = sum(possession_percentages.values())
        if total_percentage != 100.0 and total_percentage > 0:
            # Normalize to ensure they sum to 100%
            for team_side in possession_percentages:
                possession_percentages[team_side] = round(
                    (possession_percentages[team_side] / total_percentage) * 100, 2)
        
        return possession_percentages
    
    def export_match_stats(self, filename='match_stats.csv'):
        print("\n[DEBUG] Exporting match stats:")
        poss_percentages = self.calculate_possession_percentage()
        print(f"Home team: {self.shots['home']} shots, {self.game_engine.score['home']} goals, {poss_percentages['home']}% possession")
        print(f"Away team: {self.shots['away']} shots, {self.game_engine.score['away']} goals, {poss_percentages['away']}% possession")
        
        file_exists = os.path.isfile(filename)
        with open(filename, 'a', newline='') as csvfile:
            fieldnames = ['metric', 'home_value', 'away_value']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists or os.path.getsize(filename) == 0:
                writer.writeheader()
            
            # Get team strategies
            home_strategy = getattr(self.home_team, 'strategy_type', 'unknown')
            away_strategy = getattr(self.away_team, 'strategy_type', 'unknown')
            
            writer.writerow({
                'metric': 'Team Strategy',
                'home_value': home_strategy,
                'away_value': away_strategy
            })
            
            writer.writerow({
                'metric': 'Possession (%)',
                'home_value': poss_percentages.get('home', 0),
                'away_value': poss_percentages.get('away', 0)
            })
            
            writer.writerow({
                'metric': 'Total Shots On Target',
                'home_value': self.shots['home'],
                'away_value': self.shots['away']
            })
            
            writer.writerow({
                'metric': 'Goals',
                'home_value': self.game_engine.score['home'],
                'away_value': self.game_engine.score['away']
            })
        
        self._export_shot_details()
        print(f"[SUCCESS] Match stats exported to {filename}")

    def _export_shot_details(self, filename='shot_details.csv'):
        file_exists = os.path.isfile(filename)
        with open(filename, 'a', newline='') as csvfile:
            fieldnames = ['team', 'player', 'x_position', 'y_position', 'result', 'timestamp']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not file_exists or os.path.getsize(filename) == 0:
                writer.writeheader()
            for team_side in ['home', 'away']:
                for shot in self.shot_details[team_side]:
                    writer.writerow({
                        'team': team_side,
                        'player': shot.get('player', 'striker'),
                        'x_position': shot['position'][0],
                        'y_position': shot['position'][1],
                        'result': shot['result'],
                        'timestamp': shot['timestamp']
                    })


def modify_game_engine(game_engine, match_stats_tracker):
    """
    Modify the game engine to integrate match stats tracking.
    Focus ONLY on:
    1. Goals scored
    2. Goalkeeper interceptions from opponents
    3. Accurate possession tracking
    """
    # Store original methods
    original_check_for_goal = game_engine.check_for_goal
    original_check_player_ball_collision = game_engine.check_player_ball_collision
    original_update = game_engine.update
    
    def modified_check_for_goal(ball):
        """Record a shot whenever a goal is scored"""
        # Get the result from the original function
        goal_result = original_check_for_goal(ball)
        
        # If a goal was scored, record it as a shot
        if goal_result in ['home', 'away']:
            match_stats_tracker.record_shot(goal_result, 'goal', 
                                          position=ball.position.tolist() if hasattr(ball, 'position') else None)
            print(f"[GOAL] Recorded shot for {goal_result} team")
        
        return goal_result
    
    def modified_check_player_ball_collision(player, teams, ball):
        """Record a shot when a goalkeeper intercepts the ball from opponent"""
        # Remember previous owner before collision
        previous_owner = ball.owner
        previous_team = None
        has_ball_velocity = hasattr(ball, 'velocity') and ball.velocity is not None
        ball_is_moving = has_ball_velocity and np.linalg.norm(ball.velocity) > 0.5
        
        if previous_owner:
            previous_team = previous_owner.team.team_side
        
        # Call original collision handler
        original_check_player_ball_collision(player, teams, ball)
        
        # Check if this was a goalkeeper interception
        # Conditions:
        # 1. Player is a goalkeeper
        # 2. The ball is now owned by the goalkeeper
        # 3. The ball came from the opposite team OR ball was moving
        # 4. The ball wasn't previously owned by the goalkeeper's team
        if (player.role == 'goalkeeper' and 
            player == ball.owner and  # Goalkeeper now has the ball
            ball_is_moving and  # Ball was moving
            (previous_team is None or previous_team != player.team.team_side)):  # Not from same team
            
            # This counts as a shot by the opposing team
            opposing_team = 'away' if player.team.team_side == 'home' else 'home'
            match_stats_tracker.record_shot(opposing_team, 'intercepted', 
                                          position=ball.position.tolist() if hasattr(ball, 'position') else None)
            print(f"[INTERCEPTION] Recorded shot for {opposing_team} team (goalkeeper interception)")
    
    def modified_update(pitch, teams, ball, dt):
        """Update match statistics including possession tracking"""
        # Before updating game state, check if score changed from previous frame
        current_score = game_engine.score.copy()
        for team_side in ['home', 'away']:
            if current_score[team_side] > match_stats_tracker.previous_score[team_side]:
                # A goal was scored, record shots if not already recorded by check_for_goal
                goals_diff = current_score[team_side] - match_stats_tracker.previous_score[team_side]
                for _ in range(goals_diff):
                    # Check if this team's shots are less than their goals (which shouldn't happen)
                    if match_stats_tracker.shots[team_side] < current_score[team_side]:
                        match_stats_tracker.record_shot(team_side, 'goal')
                        print(f"[GOAL-CORRECTION] Added missing shot for {team_side} team")
        
        # Update previous score
        match_stats_tracker.previous_score = current_score.copy()
        
        # Update match stats (possession tracking happens here)
        match_stats_tracker.update(ball, dt)
        
        # Call original update
        original_update(pitch, teams, ball, dt)
    
    # Replace the original methods with our wrapped versions
    game_engine.check_for_goal = modified_check_for_goal
    game_engine.check_player_ball_collision = modified_check_player_ball_collision
    game_engine.update = modified_update
    
    print("[SETUP] Game engine modified to track goals, goalkeeper interceptions, and possession")


def finalize_match_stats(match_stats_tracker):
    """
    Final check and export of match statistics
    """
    print("\n[INFO] Finalizing match statistics...")
    
    # Ensure shots >= goals for each team (defensive check)
    for team_side in ['home', 'away']:
        goals = match_stats_tracker.game_engine.score[team_side]
        shots = match_stats_tracker.shots[team_side]
        
        if shots < goals:
            # This shouldn't happen, but add shots to match goals if needed
            shots_to_add = goals - shots
            print(f"[WARNING] Adding {shots_to_add} missing shots to {team_side} team to match goal count")
            for _ in range(shots_to_add):
                match_stats_tracker.record_shot(team_side, 'goal')
    
    match_stats_tracker.export_match_stats()
    print("[COMPLETE] Match stats exported successfully!")