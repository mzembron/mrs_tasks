from mrs_main.common.objects import IntrestDescription, TaskData

class KnowledgeBase:
    """ Knowledge base - class responsible for managing information about the environment agent 
        operates in, the tasks it is supposed to perform and its own abilities. """
    def __init__(self, agent_type: int):
        self._agent_type = agent_type

    def get_interst_desc(task_data: TaskData) -> IntrestDescription:
        """ Returns the interest description for the given task """
        return IntrestDescription(execution=0.2, coordination=0.2)