import threading
import typing

import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
import numpy.typing as npt
import rclpy
from mrs_msgs.msg import TasksStatesDeclaration
from rclpy.subscription import Subscription
from rclpy.node import Node


class Example_Node(Node):
    """Example Node for showing how to use matplotlib within ros 2 node
    
    Attributes:
        fig: Figure object for matplotlib
        ax: Axes object for matplotlib
        x: x values for matplotlib
        y: y values for matplotlib
        lock: lock for threading
        _sub: Subscriber for node
    """
    ROBOT_MAP = {'tb1': 0, 'tb2': 1, 'tb3': 2, 'tb4': 3, 'tb5': 4, 'tb6': 5}
    STATE_MAP = {'init': 0, 'DefineTaskIntrest': 0, 'WaitForExec': 1, 'ExecTask': 2, 'SuperviseTask': 3, 'TaskCompleted': 4 }
    def __init__(self):
        """Initialize."""
        super().__init__("example_node")
        # Initialize figure and axes and save to class
        self.fig, self.ax = plt.subplots()
        # create Thread lock to prevent multiaccess threading errors
        self._lock = threading.Lock()
        # create initial values to plot
        self.matrix_size = (3, 6)

        # Initialize the figure and axis
        self.matrix = np.zeros(self.matrix_size)

        # Add text annotations for each cell
        self.text_annotations = [[self.ax.text(j, i, '', ha='center', va='center', color='white') 
                            for j in range(self.matrix_size[1])] 
                            for i in range(self.matrix_size[0])]
        # create subscriber
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._sub: Subscription = self.create_subscription(
            TasksStatesDeclaration, '/mrs_main/tasks_states_declaration', self._callback, 10, callback_group=self.cbg
        )

    def _callback(self, msg: TasksStatesDeclaration):
        """Callback for subscriber"""
        # lock thread
        with self._lock:
            # update values
            robot_idx = self.ROBOT_MAP[msg.robot_name]
            print(f"robot_idx: {robot_idx}")
            for idx, state_name in enumerate(msg.tasks_states):
                # if idx>5:
                #     return
                print(f"state_name: {state_name}, idx: {idx}")
                self.text_annotations[robot_idx][idx-4].set_text(state_name)
                self.matrix[robot_idx][idx-4] = self.STATE_MAP[state_name]

    def plt_func(self, _):
        """Function for for adding data to axis.

        Args:
            _ : Dummy variable that is required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        # lock thread
        with self._lock:
            # new_matrix = np.random.rand(*self.matrix_size)  # Generate random values for the matrix
            self.im.set_data(self.matrix)
            return self.im, *[text for row in self.text_annotations for text in row]

    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.im = self.ax.imshow(self.matrix, cmap='viridis', vmin=0, vmax=4)
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=1000)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = Example_Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node._plt()


if __name__ == "__main__":
    main()