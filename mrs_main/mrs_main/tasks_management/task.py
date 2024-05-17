
class Task():
    def __init__(self, task_type: str, data: list[str], time_stamp, short_id: int = None):
        self.task_type: str = task_type
        self.data: list[str] = data
        self.short_id: int = short_id
        self.time_stamp = time_stamp
