import sqlite3
import os
import rospy
from mrs_msgs.srv import TaskScore, TaskScoreResponse

DIRNAME = os.path.dirname(__file__)
DB_DIR = os.path.join(DIRNAME, 'knowledge_database.db')

DEFAULT_CORRELATION = 0.5
MIN_CORRELATION = 0.1
MAX_CORRELATION = 1


class KnowledgeBaseHandler:
    def __init__(self) -> None:
        self.service  = rospy.Service('plan_master/score_task', TaskScore, self.score_task)
        self.tasks_to_be_scored = []

    def score_task(self, req):
        print(req.TaskId, req.score)
        output = "Task not found!"
        for tsk in self.tasks_to_be_scored:
            if tsk.id == req.TaskId:
                print("Scoring")
                out_correlation = self._update_correlation_for_tsk(tsk, req.score)
                output = "done!"
        return TaskScoreResponse(output)

    def get_avg_correlation(self, robot_name, tag_list):
        corr_sum = 0
        print(tag_list)
        for tag in tag_list:
            corr_sum += self.get_correlation(robot_name, tag)

        return (corr_sum/len(tag_list))


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
        # print(__file__)
        response = cursor.execute(query, (robot_name, tag_name, ))
        output = response.fetchone()
        conn.close()
        if((type(output) is not type(None)) and len(output)==1):
            return output[0]
        else:
            #  just make sure to not add
            #  more than one correlation
            assert(type(output) is type(None))
            self._insert_correlation_to_kb(DEFAULT_CORRELATION, robot_name, tag_name)
            return self.get_correlation(robot_name, tag_name)

    def _update_correlation_for_tsk(self, task, score):
        output = []
        for tag in task.tags:
            print(task.assigned_robot_name, score)
            output.append( self._update_correlation(task.assigned_robot_name, tag, score))
            print(output)

        return output

    def _update_correlation(self, robot_name, tag_name, score):
        old_correlation = self.get_correlation(robot_name, tag_name)
        new_correlation = self._calculate_new_correlation(old_correlation, score)
        robot_id = self._get_robot_id_by(robot_name)
        tag_id = self._get_tag_id_by(tag_name)
        query = 'UPDATE Compatibilities SET\
            Correlation = ? WHERE robotid = ? AND TagID = ?'
        conn = sqlite3.connect(DB_DIR)
        cursor = conn.cursor()
        cursor.execute(query, (new_correlation, robot_id, tag_id))
        conn.commit()
        return new_correlation



    def _calculate_new_correlation(self, old_correaltion, score):
        return min([MAX_CORRELATION, max([MIN_CORRELATION, (0.5*score - 0.5)*old_correaltion])])

    def _insert_robot_to_kb(self, name):
        # INSERT INTO tags (name) values ("hall")
        query = 'INSERT INTO robots (name) values (?)'
        params = (name, )
        return self._insert(query, params)
        

    def _insert_tag_to_kb(self, name):
        # INSERT INTO tags (name) values ("hall")
        query = 'INSERT INTO tags (name) values (?)'
        params = (name, )
        return self._insert(query, params)

    def _insert(self, query, params):
        conn = sqlite3.connect(DB_DIR)
        cursor = conn.cursor()
        cursor.execute(query, params)
        conn.commit()
        conn.close()
        return cursor.lastrowid

    def _insert_correlation_to_kb(self, corelation, robot_name, tag_name):
        query = 'INSERT INTO Compatibilities (Correlation, RobotID, TagID) values (?, ?,?)'
        robot_id = self._get_robot_id_by(robot_name)
        tag_id = self._get_tag_id_by(tag_name)
        params = (corelation, robot_id, tag_id, )
        return self._insert(query, params)

    def _get_robot_id_by(self, name):
        query = 'SELECT RobotID from Robots WHERE name=?'
        conn = sqlite3.connect(DB_DIR)
        cursor = conn.cursor()
        response = cursor.execute(query, (name, ))
        output = response.fetchone()
        conn.close()
        if((type(output) is not type(None)) and len(output)==1):
            return output[0]
        else:
            return self._insert_robot_to_kb(name)

    def _get_tag_id_by(self, name):
        query = 'SELECT TagID from Tags WHERE name=?'
        conn = sqlite3.connect(DB_DIR)
        cursor = conn.cursor()
        response = cursor.execute(query, (name, ))
        output = response.fetchone()
        conn.close()
        if((type(output) is not type(None)) and len(output)==1):
            return output[0]
        else:
            return self._insert_tag_to_kb(name)