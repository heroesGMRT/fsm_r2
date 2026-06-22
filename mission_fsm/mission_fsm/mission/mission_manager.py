class MissionManager:

    def __init__(self):

        self.current_area = 1

    def next_area(self):

        self.current_area += 1

    def mission_complete(self):

        return self.current_area > 3