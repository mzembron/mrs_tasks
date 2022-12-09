
from geometry_msgs.msg import PoseStamped

class DataManipulation():

    @staticmethod
    def copy_pose_data(copy_destination_data, source_data) -> None:
        #const header 'map' may be problematic
        copy_destination_data.header.frame_id = 'map'

        copy_destination_data.pose.position.x = source_data.pose.position.x
        copy_destination_data.pose.position.y = source_data.pose.position.y

        copy_destination_data.pose.orientation.x = source_data.pose.orientation.x
        copy_destination_data.pose.orientation.y = source_data.pose.orientation.y
        copy_destination_data.pose.orientation.z = source_data.pose.orientation.z
        copy_destination_data.pose.orientation.w = source_data.pose.orientation.w

    @staticmethod
    def parse_coordinates_to_task_data(coordinates) -> PoseStamped:
        # TODO move it to task !!!!
        task_data = PoseStamped()
        task_data.pose.position.x = coordinates[0]
        task_data.pose.position.y = coordinates[1]
        task_data.pose.position.z = coordinates[2]
   
        task_data.pose.orientation.x = coordinates[3]
        task_data.pose.orientation.y = coordinates[4]
        task_data.pose.orientation.z = coordinates[5]
        task_data.pose.orientation.w = coordinates[6]
        return task_data