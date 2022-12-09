import threading

import rospy
import time
from constants.constants import DELAY_TASK_PENALTY

from plan_master.turtlebot import Turtlebot
from plan_master.task.subtask import Subtask
from plan_master.task.task_execution_estimation import TaskExecutionEstimation
from move_base_msgs.msg import MoveBaseActionResult

from mrs_msgs.msg import TaskBacklog, TaskStatus


CURRENT_TASK_INDEX = 0
# possible robot status:


# move to consts
STATUS_READY = 0
STATUS_RUNNING = 1
STATUS_IDLE_BEFORE_TASK_START = 2
STATUS_IDLE_AFTER_TASK_END = 3

ACTION_SUCCEEDED = 3


class RobotTaskHarmonizer:
    """ Wrapper for robot class with purpose of managing tasks """
    def __init__(self, robot_name):
        """
        Just assume we use only turtlebots:"""
        self.robot = Turtlebot(robot_name)
        rospy.Subscriber(
            robot_name+"/move_base/result",
            MoveBaseActionResult,
            self._action_result_callback)
        self.tasks_backlog_publisher = \
            rospy.Publisher(
                "plan_master/tasks_backlog",
                TaskBacklog,
                queue_size=10)

        # Debug! trying with scenarios!
        self.task_status_publisher = \
            rospy.Publisher(
                "plan_master/scenarios_conditions",
                TaskStatus,
                queue_size=10)

        self.backlog_publishing_thread = \
            threading.Thread(
                target=self.backlog_updater,
                daemon=True)
        self.backlog_publishing_thread.start()
        self.robot_name = robot_name
        self.robot_status = STATUS_READY
        self.task_list = []


    def __del__(self):
        self.backlog_publishing_thread.join()

    # def get_estimated_task_cost_with_scheduled_position(self, new_task):
    #     """ create new class that will be returned, requirements:
    #      - cost,
    #      - scheduled position in backlog,
    #      - predicted task execution time (time of executing only this task)
    #      - predicted task start time, task end time
    #         (to determine period of time for executing this task)

    #     """
    #     full_cost = 0
    #     was_new_task_included = False
    #     task_position = -1  # initial position is last in task list
    #     if len(self.task_list) == 0:
    #         task_position = 0
    #         return self.robot.calc_cost_from_curr_position_to_spec_position(
    #             new_task.data), task_position

    #     for task_index in range(0, len(self.task_list)):
    #         task = self.task_list[task_index]
    #         task_dealy_coeficient = \
    #             self._calculate_delay_coeficient(task, new_task)

    #         if task_index == 0:
    #             full_cost += self.robot. \
    #                 calc_cost_from_curr_position_to_spec_position(
    #                     task.data)/task_dealy_coeficient

    #         elif (new_task.has_higher_priority_than(task)) and \
    #                 not was_new_task_included:
    #             # if new task has higher priority than current task including
    #             # new task into estimated cost
    #             task_position = task_index
    #             full_cost += self._calculate_cost_of_including_new_task(
    #                 task_index, new_task)
    #             was_new_task_included = True

    #         else:
    #             previous_task = self.task_list[task_index-1]
    #             full_cost += self.robot. \
    #                 calc_cost_from_spec_position_to_spec_position(
    #                     previous_task.data, task.data)/task_dealy_coeficient

    #     if was_new_task_included is False:
    #         last_task = self.task_list[-1]
    #         full_cost += self.robot. \
    #             calc_cost_from_spec_position_to_spec_position(
    #                 last_task.data,
    #                 new_task.data)

    #     # print(self.robot_name, full_cost)
    #     return full_cost, task_position


    def get_scenario_execution_estimation(self, new_tasks_list):

        ## after merge rename to get_task_execution_estiamtion
        ## Debug method for harmonizing scenarios
        """ create new class that will be returned, requirements:
            - cost,
            - scheduled position in backlog,
            - predicted task execution time (time of executing only this task)
            - predicted task start time, task end time
            (to determine period of time for executing this task)

        """
        new_tasks_list = self._prepare_task_for_estimation(new_tasks_list)
        # full_cost = 0
        was_new_task_included = False
        # task_position = -1  # initial position is last in task list
        REPRESENTANT_INDEX = 0
        # if no tasks in backlog:
        if len(self.task_list) == 0:
            return self._generate_scenario_exec_estimation(new_tasks_list)
        
        # including in betweeen current tasks
        full_cost_of_scenario = 0
        execution_estimation_list = []
        for task_idx, task in enumerate(self.task_list):
            
            ## calculate whether new tasks are going to move this task
            ## (beacuse of highier priority)
            task_dealy_coeficient = \
                self._calculate_delay_coeficient(task, new_tasks_list[REPRESENTANT_INDEX])

            # we do not disturb current task (no interuption while executing task so far)
            if task_idx == 0:
                print("========= Cost before adding first task: ", full_cost_of_scenario, " =========")
                full_cost_of_scenario += self.robot. \
                    calc_cost_from_curr_position_to_spec_position(
                        task.data)/task_dealy_coeficient
                print("========= Cost after adding first task: ", full_cost_of_scenario, " =========")
            elif (new_tasks_list[REPRESENTANT_INDEX].has_higher_priority_than(task)) and \
                    not was_new_task_included:
                # if new task has higher priority than current task -> including
                # new task into estimated cost
                # full_cost_of_scenario += self._calculate_cost_of_including_scenario(
                #     task, task_idx , new_tasks_list)
                execution_estimation_list = self._estimate_including_scenario(
                    task,
                    task_idx,
                    new_tasks_list,
                    full_cost_of_scenario
                )
                was_new_task_included = True

            else:
                # include cost of task in backlog
                previous_task = self.task_list[task_idx-1]
                task_cost = self.robot. \
                    calc_cost_from_spec_position_to_spec_position(
                        previous_task.data, task.data)/task_dealy_coeficient
                full_cost_of_scenario += task_cost

                if(len(execution_estimation_list)>0):
                    #update estimated costs
                    self._update_cost_for_scenario(execution_estimation_list, task_cost)

        # including at the end of current tasks
        if was_new_task_included is False:
            assert(len(execution_estimation_list)==0)
            last_task = self.task_list[-1]
            starting_index = len(self.task_list) # index of 

            print("========= Cost before adding first task: ", full_cost_of_scenario, " =========")
            execution_estimation_list = self._generate_scenario_exec_estimation(
                new_tasks_list,
                starting_index,
                full_cost_of_scenario,
                last_task
            )
            print("========= Cost after adding first task: ", execution_estimation_list[0].full_cost, " =========")

        # make some debug print

        # print(self.robot_name, full_cost)
        
        return execution_estimation_list

    def receive_scenario_signal(self):
        print("[ ",self.robot_name, " ]"," received signal - continuing task execution")
        if (self.robot_status == STATUS_IDLE_BEFORE_TASK_START):
            # distinguish idle befor start and after ending
            self.order_task()
        elif (self.robot_status == STATUS_IDLE_AFTER_TASK_END):
            self._handle_unfinished_task()


    def add_task(self, task, position):
        if len(self.task_list) == 0:
            self.task_list.append(task)
            self.order_task()
        elif position == -1 or position > len(self.task_list)-1:
            self.task_list.append(task)
        else:
            self.task_list.insert(position, task)

    def order_task(self):
        if (len(self.task_list) != 0):
            # Debug!
            current_task = self.task_list[0]
            if ((type(current_task) is Subtask) and (not current_task.is_start_req_met())) :
                self.robot_status = STATUS_IDLE_BEFORE_TASK_START
                # print(" [ ", self.robot_name)
                print(" [ ", self.robot_name, " ] ", "condition to start task not met. Going into idle mode and waiting!")
                return 
            self.robot.go_to_goal(current_task.data)
            self.robot_status = STATUS_RUNNING

            print(" [ ", self.robot_name, " ] ", " status: ", self.robot_status)

    def backlog_updater(self):
        while (not rospy.is_shutdown()):
            time.sleep(5)
            backlog = TaskBacklog()
            backlog.robot_name = self.robot_name
            for task in self.task_list:
                backlog.tasks.append(task.return_task_desc_msg())

            self.tasks_backlog_publisher.publish(backlog)

    # def _calculate_cost_of_including_new_task(self, curr_task_index, new_task):
    #     cost = 0
    #     previous_task = self.task_list[curr_task_index-1]
    #     curr_task = self.task_list[curr_task_index]
    #     cost += self.robot.calc_cost_from_spec_position_to_spec_position(
    #         previous_task.data, new_task.data)
    #     cost += self.robot.calc_cost_from_spec_position_to_spec_position(
    #         new_task.data, curr_task.data)/DELAY_TASK_PENALTY
    #     return cost

    def _prepare_task_for_estimation(self, task_or_task_list):
        if(type(task_or_task_list) is list):
            return task_or_task_list
        else:
            return [task_or_task_list]
##### Additional methods for planning scenarios

    def _generate_scenario_exec_estimation(self, new_tasks_list, starting_idx=0, 
        starting_cost=0, prev_task=None):

        execution_estimation_list = []
        cost = starting_cost
        print("== len of new taks: ", len(new_tasks_list), " ==")
        for idx, task in enumerate(new_tasks_list):
            single_execution_estimation = TaskExecutionEstimation()
            single_execution_estimation.task_position = idx + starting_idx
            if (idx==0 and starting_idx==0):
                cost += self.robot.calc_cost_from_curr_position_to_spec_position(
                    task.data
                )
            elif (idx == 0 and starting_idx != 0):
                assert(prev_task is not None)
                print("==== before ading last task cost: ", cost, " ====")
                cost += self.robot.calc_cost_from_spec_position_to_spec_position(
                    prev_task.data, 
                    task.data
                )
                print("==== after ading last task cost: ", cost, " ====")
            else:
                prev_scenario_task = new_tasks_list[idx-1]
                cost += self.robot.calc_cost_from_spec_position_to_spec_position(
                    prev_scenario_task.data, 
                    task.data
                )

            single_execution_estimation.full_cost = cost 
            execution_estimation_list.append(
                single_execution_estimation
            )
        print("task cos: ",execution_estimation_list[0].full_cost)
        return execution_estimation_list

    def _estimate_including_scenario(self, curr_task, curr_task_idx, new_tasks_list, curr_cost):
        prev_task = self.task_list[curr_task_idx-1]
        #including moved task cost 
        temp_cost = curr_cost + self.robot.calc_cost_from_spec_position_to_spec_position(
                    new_tasks_list[-1].data, 
                    curr_task.data
                )/DELAY_TASK_PENALTY
        execution_estimation_list = self._generate_scenario_exec_estimation(
            new_tasks_list,
            starting_idx=curr_task_idx,
            starting_cost=temp_cost,
            prev_task=prev_task
        )
        return execution_estimation_list

    def _update_cost_for_scenario(self, estimations_list, additional_cost):
        for estimation in estimations_list:
            estimation.full_cost += additional_cost
##### End Additional methods for planning scenarios

    def _calculate_delay_coeficient(self, curr_task, new_task):
        task_dealy_coeficient = 1
        if new_task.priority < curr_task.priority:
            # lower number highier priority
            task_dealy_coeficient = DELAY_TASK_PENALTY
        return task_dealy_coeficient

    def _action_result_callback(self, result):
        # Debug!
        # print(self.robot.current_goal)
        self.robot.current_goal = None # rethink if current goal is needed!!
        current_task = self.task_list[CURRENT_TASK_INDEX]
        task_status = TaskStatus()
        task_status.status = result.status.status
        task_status.id.id = current_task.id
        if (result.status.status == ACTION_SUCCEEDED):
            print("[ " ,self.robot_name, " ]", " achieved goal! #############")
            ## publish output for all tasks 
            ## scenarios debug!
            if  (type(current_task) is Subtask):
                task_status.id.index = current_task.index
                if (not current_task.is_end_req_met()):
                    self.robot_status = STATUS_IDLE_AFTER_TASK_END
                  # remember taks statuts
                    self._unfinished_task_status = task_status
                    print("Task can not be finished currently - going into idle state and waiting!")
                    return  # end callback!
            else:
                task_status.id.index = 0 # as ordinary task does not have subtasks

            ## here take care of end cases !!!
            if (len(self.task_list) != 0):
                self.task_list.pop(CURRENT_TASK_INDEX)
                self.order_task()
        else:
            # TODO advertise task to plan master
            # Here robot should go to the next task !!!!
            print("[ " ,self.robot_name, " ]", " was unable to achive goal! #############")

        self.task_status_publisher.publish(task_status)


    def _handle_unfinished_task(self):
        current_task = self.task_list[0]
        if (current_task.is_end_req_met()):
            assert(self._unfinished_task_status is not None)
            self.task_status_publisher.publish(self._unfinished_task_status)
            self._unfinished_task_status = None
            self.task_list.pop(CURRENT_TASK_INDEX)
            self.order_task()

    def _was_task_delayed(self, task_dealy_coeficient):
        return task_dealy_coeficient == DELAY_TASK_PENALTY
