class TaskExecutionEstimation:
    """
    TaskExecutionEstimation - contains parameters of plan on task execution
    returned by harmonizer.   
    """
    def __init__(self, full_cost=0, task_position=-1):
        """
        Constructor - no parameters necessary, every 
        parameter can be accessed after construction
        """
        self.full_cost = full_cost
        self.task_position = task_position
    