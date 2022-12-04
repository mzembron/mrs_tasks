from operator import index
import pandas as pd
import os
from helpful_functions.data_manipulation import DataManipulation
from geometry_msgs.msg import PoseStamped

class RoomCoordinatesParser:
    def __init__(self):
        dirname = os.path.dirname(__file__)
        self.rooms_coordinates = pd.read_csv(os.path.join(dirname, "rooms_coordinates.csv"), header=0, index_col = "room name")

    def get_room_coordinates(self, room_name):
        return (tuple(self.rooms_coordinates.loc[room_name,:]))

    # TODO change room to tag 
    def get_room_pose(self, room_name) -> PoseStamped:
        room_coordinates = tuple(self.rooms_coordinates.loc[room_name,:])
        return DataManipulation.parse_coordinates_to_task_data(room_coordinates)