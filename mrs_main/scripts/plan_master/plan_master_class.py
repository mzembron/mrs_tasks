import rospy
from geometry_msgs.msg import PoseStamped


from plan_master.task.task import Task
from plan_master.task.scenarios_controller \
    import ScenariosController
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

    def subscribe(self, robot):
        self.robots_harmonizers.append(robot)

    def _move_base_goal_callback(self, data):
        task = Task('GT', data, priority=8)
        self._order_task_execution(task)

    def _order_task_callback(self, task_desc):
        try:
            if(Task.is_task_type_scenario(task_desc.type)):
                # TODO take care of scenario
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
        room_coordinates = RoomCoordinatesParser() \
            .get_room_coordinates(task_desc.data[0])
        # as simple task have just one attribute
        task_data = self._prepare_task_data(room_coordinates)
        task = Task(task_desc.type, task_data, priority=task_desc.priority)
        self._order_task_execution(task)

    def _handle_scenario(self, task_desc):
        # exclude parsing of scenario data
        scenario_data = []
        for data_desc in task_desc.data:
            room_coordinates = RoomCoordinatesParser() \
                .get_room_coordinates(data_desc)
            task_data = self._prepare_task_data(room_coordinates)
            scenario_data.append(task_data)
        task_data = self._prepare_task_data(room_coordinates)
        ## leve here just this:
        scenario = Task(task_desc.type, scenario_data, task_desc.priority)
        print(self.scenarios_controller.get_tasks_groups_for_one_robot(scenario))
        for subtask in scenario.subtasks_list:
            # in here scenario planning and execution
            # just debg! take care of the first batch of tasks
            if len(subtask.required_to_start_if_met_dict) == 0:
                optimal_harmonizer, position = self._select_optimal_harmonizer(subtask)
                optimal_harmonizer.add_task(subtask, position)
                self.scenarios_controller\
                    .subtask_harmonizer_dict[subtask] = optimal_harmonizer


    def _order_task_execution(self, task):
        optimal_harmonizer, position = self._select_optimal_harmonizer(task)
        optimal_harmonizer.add_task(task, position)

    def _select_optimal_harmonizer(self, task):
        harmonizer_cost_dict = {}
        harmonizer_task_position_dict = {}
        for harmonizer in self.robots_harmonizers:
            if (task.is_suitable_for_robot(harmonizer.robot)):
                harmonizer_cost_dict[harmonizer], harmonizer_task_position_dict[harmonizer] \
                = harmonizer.get_estimated_task_cost_with_scheduled_position(task)
                print("[ Estimated cost for: ", harmonizer.robot_name,
                    "] ", harmonizer_cost_dict[harmonizer],
                    "Task at position: ", harmonizer_task_position_dict[harmonizer])

        optimal_robot_harmonizer = min(harmonizer_cost_dict, key=harmonizer_cost_dict.get) 
        return optimal_robot_harmonizer, harmonizer_task_position_dict[optimal_robot_harmonizer]

    def _prepare_task_data(self, coordinates):
        # TODO move it to task !!!!
        task_data = PoseStamped()
        task_data.pose.position.x = coordinates[0]
        task_data.pose.position.y = coordinates[1]
        task_data.pose.position.z = coordinates[2]
   
        task_data.pose.orientation.x = coordinates[3]
        task_data.pose.orientation.y = coordinates[4]
        task_data.pose.orientation.z = coordinates[5]
        task_data.pose.orientation.w = coordinates[6]
        return task_data

