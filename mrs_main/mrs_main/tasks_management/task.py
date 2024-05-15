
class Task():
    def __init__(self, task_type: str, data: list[str], time_stamp, priority: int = None):
        self.task_type: str = task_type
        self.data: list[str] = data
        self.priority: int = priority
        self.time_stamp = time_stamp
