class TaskTypeNotIntroducedError(Exception):
    def __init__(self):
        self.message = "Task type was not yet introduced!"
        super().__init__(self.message)