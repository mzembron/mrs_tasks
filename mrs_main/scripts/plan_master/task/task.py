
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

    