import rospy
from geometry_msgs.msg import PoseStamped


from plan_master.task.task import Task
from plan_master.task.task_type_not_introduced_error \
    import TaskTypeNotIntroducedError
from mrs_msgs.msg import TaskDesc
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
        # subscription of task on general topic like /plan_master/ordered_tasks
        self.robots_harmonizers = []

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
                
        except(KeyError):
            print("Room name does not exist in system!")

        except TaskTypeNotIntroducedError as type_task_error:
            print(type_task_error.message)

    def _handale_simple_task(self, task_desc):
        room_coordinates = RoomCoordinatesParser() \
            .get_room_coordinates(task_desc.data[0])
        # as simple task have just one attribute
        task_data = self._prepare_task_data(room_coordinates)
        task = Task(task_desc.type, task_data, priority=task_desc.priority)
        self._order_task_execution(task)

    def _handle_scenario(self, task_desc):
        pass

    def _order_task_execution(self, task):
        optimal_robot, position = self._select_optimal_robot(task)
        optimal_robot.add_task(task, position)

    def _select_optimal_robot(self, task):
        robots_cost_dict = {}
        robots_and_task_position_dict = {}
        for robot_harmonizer in self.robots_harmonizers:
            if(task.is_suitable_for_robot(robot_harmonizer.robot)):
                robots_cost_dict[robot_harmonizer], robots_and_task_position_dict[robot_harmonizer] = robot_harmonizer.get_estimated_task_cost_with_scheduled_position(task)
                print("[ Estimated cost for: ", robot_harmonizer.robot_name, "] ", robots_cost_dict[robot_harmonizer],
                    "Task at position: ", robots_and_task_position_dict[robot_harmonizer])

        optimal_robot_harmonizer = min(robots_cost_dict, key=robots_cost_dict.get) 
        return optimal_robot_harmonizer, robots_and_task_position_dict[optimal_robot_harmonizer]

    def _prepare_task_data(self, coordinates):
        task_data = PoseStamped()
        task_data.pose.position.x = coordinates[0]
        task_data.pose.position.y = coordinates[1]
        task_data.pose.position.z = coordinates[2]
   
        task_data.pose.orientation.x = coordinates[3]
        task_data.pose.orientation.y = coordinates[4]
        task_data.pose.orientation.z = coordinates[5]
        task_data.pose.orientation.w = coordinates[6]
        return task_data

