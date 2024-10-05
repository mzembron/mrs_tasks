from abc import ABC, abstractmethod

class AbstractExecutor(ABC):
    """ Interface for task executor allowing to support new types of agents """

    def __init__(self, callback_on_finish):
        """ Initialize the executor with a callback function to be called on finish """
        self.callback_on_finish = callback_on_finish
    @abstractmethod
    def start_execution(self):
        """ Entrypoint to trigger the execution of specific task  """
        pass

    @abstractmethod
    def get_execution_info(self):
        """ Retrive the current status of the task execution """
        pass