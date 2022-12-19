import rospy
from geometry_msgs.msg import PoseStamped


from plan_master.task.task import Task
from plan_master.task.scenarios_controller \
    import ScenariosController
from plan_master.task_harmonizer import \
    TaskHarmonizer

from plan_master.knowledge_base.\
    knowledge_base_handler import KnowledgeBaseHandler
from plan_master.task.task_type_not_introduced_error \
    import TaskTypeNotIntroducedError
from mrs_msgs.msg import TaskDesc, TaskStatus
from data_parser.room_coordinates_parser import RoomCoordinatesParser

class PlanMaster():
    """Main system planner class."""

    def __init__(self):
        """"""
        rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self._move_base_goal_callback)
        rospy.Subscriber(
            "plan_master/order_task",
            TaskDesc,
            self._order_task_callback)

        rospy.Subscriber(
            "plan_master/scenarios_conditions",
            TaskStatus,
            self._task_status_callback)
        # subscription of task on general topic like /plan_master/ordered_tasks
        self.robots_harmonizers = []
        self.scenarios_controller = ScenariosController(self.robots_harmonizers)
        self.knowledge_base_handler = KnowledgeBaseHandler()

    def subscribe(self, robot):
        task_harmonizer = TaskHarmonizer(robot)
        """ add the robot to be managed by Plan Master """
        self.robots_harmonizers.append(task_harmonizer)

    def _move_base_goal_callback(self, data):
        task = Task('GT', data, priority=8)
        self._order_task_execution(task)

    def _order_task_callback(self, task_desc):
        try:
            if(Task.is_task_type_scenario(task_desc.type)):
                self._handle_scenario(task_desc)

            elif(Task.does_task_type_exists_in_system(task_desc.type)):
                self._handale_simple_task(task_desc)

            else: 
                raise TaskTypeNotIntroducedError()
                
        # except(KeyError):
            # create specific error for room name (or tag) not found!
        #     print("Room name does not exist in system!")

        except TaskTypeNotIntroducedError as type_task_error:
            print(type_task_error.message)

    def _task_status_callback(self, task_status):
        if(str(task_status.id.id).endswith('scenario')):
            self.scenarios_controller.handle_completed_subtask(task_status)
        else:
            print('Simple task has ended!')
        # print("[ INFO ] Got that task of id: ", task_status.id.id, " and index: ",
        # task_status.id.index, " has ended" )

    def _handale_simple_task(self, task_desc):
        task_data = RoomCoordinatesParser() \
            .get_room_pose(task_desc.data[0])
        task = Task(task_desc.type, task_data, priority=task_desc.priority)
        self._order_task_execution(task)

    def _handle_scenario(self, task_desc):
        # exclude parsing of scenario data
        scenario_data = []
        for data_desc in task_desc.data:
            task_data = RoomCoordinatesParser() \
                .get_room_pose(data_desc)
            scenario_data.append(task_data)
        ## leve here just this:
        scenario = Task(task_desc.type, scenario_data, task_desc.priority)

        # grouping subtasks
        subtasks_group_dict = self.scenarios_controller.get_tasks_groups_for_one_robot(scenario)
        self._order_scenario_subtasks(subtasks_group_dict)

    def _select_optimal_harmonizer(self, subtask_list):
        print("####################")
        print(self.knowledge_base_handler.get_correlation("/robot3_dirty","kitchen"))
        print("####################")

        harmonizer_estimation_dict = {}
        for harmonizer in self.robots_harmonizers:
            if (subtask_list[0].is_suitable_for_robot(harmonizer.robot)):
                estimations_list = harmonizer.get_execution_estimation_of(subtask_list)
                harmonizer_estimation_dict[harmonizer] = estimations_list
                # print("======== Scenario selection ========")
                # print("[ Estimated cost for: ", harmonizer.robot_name,
                #     "] ",  harmonizer_estimation_dict[harmonizer][0].full_cost,
                #     "Task at position: ",  harmonizer_estimation_dict[harmonizer][0].task_position)
        
        optimal_harmonizer = self._find_harmonizer_with_lowest_cost(harmonizer_estimation_dict)
        return optimal_harmonizer, harmonizer_estimation_dict[optimal_harmonizer]

    def _order_scenario_subtasks(self, subtasks_group_dict):
        # to not assign more than one group to same robot
        #### TODO very important
        # used_robots = []
        for _, subtasks_list in subtasks_group_dict.items():
            optimal_harmonizer, estimations = self._select_optimal_harmonizer(subtasks_list)

            for idx, subtask in enumerate(subtasks_list):
                task_index_in_backlog = estimations[idx].task_position
                
                optimal_harmonizer.add_task(subtask, task_index_in_backlog)
                ## adding to scenario controller !!!!
                self.scenarios_controller\
                    .subtask_harmonizer_dict[subtask] = optimal_harmonizer

    def _calculate_avg_subtask_cost(self, estimations_list):
        sum = 0
        for estimation in estimations_list:
            sum += estimation.full_cost
        return sum/len(estimations_list)

    def _find_harmonizer_with_lowest_cost(self, harmonizer_estimation_dict):
        # list(test_dict.keys())[0]
        harmonizer_cost_dict = {}
        for harmonizer, estimations in harmonizer_estimation_dict.items():
            avg_cost = self._calculate_avg_subtask_cost(estimations)
            harmonizer_cost_dict[harmonizer] = avg_cost
        optimal_harmonizer = min(harmonizer_cost_dict, key=harmonizer_cost_dict.get)
        return optimal_harmonizer

    def _order_task_execution(self, task):
        optimal_harmonizer, estimations = self._select_optimal_harmonizer([task])
        task_index_in_backlog = estimations[0].task_position
        optimal_harmonizer.add_task(task, task_index_in_backlog)

