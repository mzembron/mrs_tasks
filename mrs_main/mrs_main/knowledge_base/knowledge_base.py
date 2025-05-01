from mrs_main.common.objects import IntrestDescription, TaskData, Position
import math
ROOM_DICT = {
    1 : (-6.0, 1.0),
    2 : ( -3.0, 1.0),
    3 : (-3.0, -4.5),
    4 : (-0.5, -4.5),
    5:  (5.0, -2.0),
    6:  (2.0, 7.0)
}


class KnowledgeBase:
    """ Knowledge base - class responsible for managing information about the environment agent 
        operates in, the tasks it is supposed to perform and its own abilities. """
    def __init__(self, agent_type: int, speed:float = 1.0):
        self._agent_type = agent_type
        self._agents_number:int = 3
        self._agent_speed: float = speed
        self.current_position: Position = Position(0, 0)

    def update_position(self, x, y):
        self.current_position.x = x
        self.current_position.y = y


    def get_intrest_desc_go_to(self, task_data: TaskData, prev_task_end_time) -> IntrestDescription:
        pass

    def get_intrest_desc_search_task(self, task_data: TaskData, prev_task_end_time) -> IntrestDescription:
        if (task_data.short_id > 3 and task_data.short_id < 10):
            room_coords = ROOM_DICT[task_data.short_id-3]
            distance = math.sqrt((self.current_position.x -room_coords[0])**2 + (self.current_position.y - room_coords[1])**2)
            return IntrestDescription(distance + prev_task_end_time, distance + prev_task_end_time)


        if (self._agent_type % 3 == 0 and task_data.short_id % 3 == 0):
            return IntrestDescription(execution=102, coordination=123)
        elif (self._agent_type % 3 == 1 and task_data.short_id % 3 == 1):
            return IntrestDescription(execution=33, coordination=33)
        elif (self._agent_type % 3 == 2 and task_data.short_id % 3 == 2):
            return IntrestDescription(execution=45, coordination=33)
        else:
            return IntrestDescription(execution=1043, coordination=1232)

    def get_intrest_desc(self, task_data: TaskData) -> IntrestDescription:
        """ Returns the interest description for the given task """
        # some dummy logic to diffrentiate between interest in tasks
        # if (self._agent_type%2 == 1 and task_data.short_id%2 == 1):
        #     return IntrestDescription(execution=0.7, coordination=0.7)
        # elif (self._agent_type%2 == 0 and task_data.short_id%2 == 0):
        #     return IntrestDescription(execution=0.7, coordination=0.7)
        # else:
        #     return IntrestDescription(execution=0.2, coordination=0.2)
        
        # if (self._agent_type % 3 == 0 and task_data.short_id % 3 == 0):
        #     return IntrestDescription(execution=0.7, coordination=0.7)
        # elif (self._agent_type % 3 == 1 and task_data.short_id % 3 == 1):
        #     return IntrestDescription(execution=0.7, coordination=0.7)
        # elif (self._agent_type % 3 == 2 and task_data.short_id % 3 == 2):
        #     return IntrestDescription(execution=0.7, coordination=0.7)
        # else:
        #     return IntrestDescription(execution=0.2, coordination=0.2)

        
        if (task_data.short_id > 3 and task_data.short_id < 10):
            room_coords = ROOM_DICT[task_data.short_id-3]
            distance = math.sqrt((self.current_position.x -room_coords[0])**2 + (self.current_position.y - room_coords[1])**2)
            return IntrestDescription(distance, distance)


        if (self._agent_type % 3 == 0 and task_data.short_id % 3 == 0):
            return IntrestDescription(execution=102, coordination=123)
        elif (self._agent_type % 3 == 1 and task_data.short_id % 3 == 1):
            return IntrestDescription(execution=33, coordination=33)
        elif (self._agent_type % 3 == 2 and task_data.short_id % 3 == 2):
            return IntrestDescription(execution=45, coordination=33)
        else:
            return IntrestDescription(execution=1043, coordination=1232)