import threading
import typing
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
import numpy.typing as npt
import rclpy
from mrs_msgs.msg import TasksStatesDeclaration
from rclpy.subscription import Subscription
from rclpy.node import Node


COLOR_MAP = ListedColormap(['white', 'lightgrey', 'gold', 'lightblue', 'lightcoral', 'lightgreen' ])

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
    STATE_MAP = {'init': 0, 'DefineEstimate': 1, 'WaitForExec': 2, 'ExecTask': 3, 'SuperviseTask': 4, 'TaskCompleted': 5 }
    def __init__(self):
        """Initialize."""
        super().__init__("example_node")
        # Initialize figure and axes and save to class
        self.fig, self.ax = plt.subplots()
        self.ax.set_xticks(np.arange(6))
        self.ax.set_xticklabels(['1', '2', '3', '4', '5', '6'])
        self.ax.set_yticks(np.arange(3))
        self.ax.set_yticklabels(['tb1', 'tb2', 'tb3'])
        self.matrix_size = (3, 6)

        # self.ax.set_yticks(np.arange(4))
        # self.ax.set_yticklabels(['tb1', 'tb2', 'tb3', 'tb4'])
        # self.matrix_size = (4, 6)


        # create Thread lock to prevent multiaccess threading errors
        self._lock = threading.Lock()
        # create initial values to plot

        # Initialize the figure and axis
        self.matrix = np.zeros(self.matrix_size)

        # Add text annotations for each cell
        self.text_annotations = [[self.ax.text(j, i, '', ha='center', va='center', color='black') 
                            for j in range(self.matrix_size[1])] 
                            for i in range(self.matrix_size[0])]
        # create subscriber
        self.cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._sub: Subscription = self.create_subscription(
            TasksStatesDeclaration, '/mrs_main/tasks_states_declaration', self._callback, 10, callback_group=self.cbg
        )
        # self.text_annotations[1][1].set_text('DefineEstimate')
        # self.matrix[1][1] = 1
        # self.text_annotations[1][2].set_text('WaitForExec') 
        # self.matrix[1][2] = 2
        # self.text_annotations[1][3].set_text('ExecTask')
        # self.matrix[1][3] = 3
        # self.text_annotations[1][4].set_text('SuperviseTask')
        # self.matrix[1][4] = 4
        # self.text_annotations[1][5].set_text('TaskCompleted')
        # self.matrix[1][5] = 5

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
                if idx > 9:
                    return
                if idx >3:
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
        self.im = self.ax.imshow(self.matrix, cmap=COLOR_MAP, vmin=0, vmax=5)
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