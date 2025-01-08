from mrs_main.common.objects import IntrestDescription, TaskData

class KnowledgeBase:
    """ Knowledge base - class responsible for managing information about the environment agent 
        operates in, the tasks it is supposed to perform and its own abilities. """
    def __init__(self, agent_type: int):
        self._agent_type = agent_type

    def get_intrest_desc(self, task_data: TaskData) -> IntrestDescription:
        """ Returns the interest description for the given task """
        # some dummy logic to diffrentiate between interest in tasks
        if (self._agent_type%2 == 1 and task_data.short_id%2 == 1):
            return IntrestDescription(execution=0.7, coordination=0.7)
        elif (self._agent_type%2 == 0 and task_data.short_id%2 == 0):
            return IntrestDescription(execution=0.7, coordination=0.7)
        else:
            return IntrestDescription(execution=0.2, coordination=0.2)