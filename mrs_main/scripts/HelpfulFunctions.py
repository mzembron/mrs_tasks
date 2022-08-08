
class HelpfulFunctions():

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