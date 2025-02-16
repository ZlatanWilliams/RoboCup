class Player:
    def __init__(self, team, role, initial_position):
        self.team = team  # 'left' or 'right'
        self.role = role  # 'goalkeeper', 'defender', 'midfielder', 'striker'
        self.position = initial_position  # (x, y) coordinates
        self.color = 'blue' if team == 'left' else 'red'
        self.size = 80  # For visualization
        self.speed = 0  # Will be used later for movement
        self.direction = 0  # Will be used later for movement
        self.owner = False

    def update_position(self, new_position):
        self.position = new_position

    def get_visual_properties(self):
        """Return properties needed for visualization"""
        return {
            'position': self.position,
            'color': self.color,
            'size': self.size,
            'label': f"{self.role[0].upper()}",  # First letter of role
            'team': self.team
        }