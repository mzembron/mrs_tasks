import sqlite3
import os
import rospy
from mrs_msgs.srv import TaskScore, TaskScoreResponse

DIRNAME = os.path.dirname(__file__)
DB_DIR = os.path.join(DIRNAME, 'knowledge_database.db')

class KnowledgeBaseHandler:
    def __init__(self) -> None:
        self.service  = rospy.Service('plan_master/score_task', TaskScore, self.score_task)

    def score_task(self, req):
        print("Scoring")
        return TaskScoreResponse("good")



# Querry i am looking for:

# SELECT c.correlation FROM 
# 	robots r join compatibilities c on (r.robotID = c.robotID)
# 			join tags t on (c.tagID = t.tagID)
# 			WHERE r.name = "/robot3_dirty" AND t.name = "kitchen";

    def get_correlation(self, robot_name, tag_name):
        query = 'SELECT c.correlation FROM robots r\
         join compatibilities c on (r.robotID = c.robotID)\
         join tags t on (c.tagID = t.tagID)\
         WHERE r.name = ? AND t.name = ?;'
        conn = sqlite3.connect(DB_DIR)
        cursor = conn.cursor()
        print(__file__)
        response = cursor.execute(query, (robot_name, tag_name, ))
        output = response.fetchone()
        conn.close()
        if((type(output) is not type(None)) and len(output)==1):
            return output[0]
        else:
            return 1.0