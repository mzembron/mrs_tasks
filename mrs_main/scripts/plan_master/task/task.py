from mrs_msgs.msg import TaskDesc

###
# Task class
# priority:
#   int from 0 to 10
#   0 - most important 
#   10 - least important
###

class Task():
    
    def __init__(self, type, data, priority = 10):
        self.type = type
        self.data = data
        self.priority = priority

    def parse_task_from_msg(self, msg):
        self.type = msg.type
        # self.data = None
        self.priority = msg.prioirty

    def return_task_desc_msg(self):
        #Debug!
        # self.data = None
        task_msg = TaskDesc()
        task_msg.data = "x: "+str(self.data.pose.position.x) + "y: " + str(self.data.pose.position.y)
        task_msg.priority = self.priority
        task_msg.type = self.type
        return task_msg