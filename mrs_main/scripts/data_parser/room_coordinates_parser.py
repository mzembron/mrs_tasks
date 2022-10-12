from operator import index
import pandas as pd
import os
class RoomCoordinatesParser:
    def __init__(self):
        dirname = os.path.dirname(__file__)
        self.rooms_coordinates = pd.read_csv(os.path.join(dirname, "rooms_coordinates.csv"), header=0, index_col = "room name")
        print(self.rooms_coordinates)

    def get_room_coordinates(self, room_name):
        return (tuple(self.rooms_coordinates.loc[room_name,:]))