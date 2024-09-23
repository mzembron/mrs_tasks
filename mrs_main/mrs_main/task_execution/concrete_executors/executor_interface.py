from abc import ABC, abstractmethod

class AbstractExecutor(ABC):
    """ Interface for task executor allowing to support new types of agents """

    @abstractmethod
    def start_execution(self):
        """ Entrypoint to trigger the execution of specific task  """
        pass

    @abstractmethod
    def get_execution_info(self):
        """ Retrive the current status of the task execution """
        pass