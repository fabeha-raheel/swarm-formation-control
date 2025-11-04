import math
import rospy

from Swarm_Drone_API import MAVROS_Drone
from SwarmFollower_API import SwarmFollower


class SwarmLeader(MAVROS_Drone):
    def __init__(self, name='drone1', n_followers=2, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self.ns = name
        self.data.header.name = name
        self.data.header.id = 1
        
        self.n_followers = n_followers
        self.followers = []
        
        rospy.init_node('Swarm_Leader')
        
        self.initialize()
        
    def initialize(self):
        # Initialize leader's ROS subscribers and publishers
        self.init_subscribers()
        self.init_publishers()

    def initialize_followers(self):
        # Create SwarmFollower instances and store them in self.followers list
        for i in range(self.n_followers):
            this_follower = SwarmFollower()
            this_follower.data.header.name = 'drone'+str(i+2)
            this_follower.ns = 'drone'+str(i+2)
            this_follower.data.header.id = i+2

            this_follower.set_stream_rate()
            
            self.followers.append(this_follower)

        # Initialize follower's ROS subscribers and publishers
        for i in range(self.n_followers):
            self.followers[i].init_subscribers()
            self.followers[i].init_publishers()
            
    def wait_for_GPS_Fix(self):
        while True:
            if self.data.global_position.gps_fix == 0:
                print("Leader GPS Fix acquired!")
                break
        while True:
            if self.check_followers_GPS_Fix():
                print("All Followers have acquired GPS Fix!")
                break
        
    def check_followers_GPS_Fix(self):
        followers_ready = 0    
        for follower in self.followers:
            if follower.data.global_position.gps_fix == 0:
                followers_ready += 1
        
        if followers_ready == self.n_followers:
            return True
        else:
            return False
    
    def calculate_line_formation_coordinates(self, offset=2):
        follower_coordinates = []
        
        for i in range(self.n_followers):
            dN = -1*offset*(i+1)
            dE = 0
            converted_offsets = self.drone_to_ned_conversion(dNorth=dN, dEast=dE, heading=self.data.euler_orientation.yaw)
            
            new_coords = self.ellipsoid_offset_location(latitude=self.data.global_position.latitude,
                                              longitude=self.data.global_position.longitude,
                                              dNorth=converted_offsets[0],
                                              dEast=converted_offsets[1])
            follower_coordinates.append(new_coords)
            
        return follower_coordinates
    
    def calculate_flock_formation_coordinates(self, offset=2):
        follower_coordinates = []
        
        if self.n_followers%2 == 0:    # if number of followers is even
            for i in range(int(self.n_followers / 2)):
                dN = -0.866*offset*(i+1)
                dE = (offset*(i+1))/2
                converted_offsets = self.drone_to_ned_conversion(dNorth=dN, dEast=dE, heading=self.data.euler_orientation.yaw)
                
                coords = self.ellipsoid_offset_location(latitude=self.data.global_position.latitude,
                                                        longitude=self.data.global_position.longitude,
                                                        dNorth=converted_offsets[0],
                                                        dEast=converted_offsets[1])
                follower_coordinates.append(coords)
                
                dE = (-1*offset*(i+1))/2
                converted_offsets = self.drone_to_ned_conversion(dNorth=dN, dEast=dE, heading=self.data.euler_orientation.yaw)
                
                coords = self.ellipsoid_offset_location(latitude=self.data.global_position.latitude,
                                                        longitude=self.data.global_position.longitude,
                                                        dNorth=converted_offsets[0],
                                                        dEast=converted_offsets[1])
                follower_coordinates.append(coords)
        
        else:                               # else if number of followers is odd
            dN = -1*offset
            dE = 0
            converted_offsets = self.drone_to_ned_conversion(dNorth=dN, dEast=dE, heading=self.data.euler_orientation.yaw)
            coords = self.ellipsoid_offset_location(latitude=self.data.global_position.latitude,
                                                    longitude=self.data.global_position.longitude,
                                                    dNorth=converted_offsets[0],
                                                    dEast=converted_offsets[1])
            follower_coordinates.append(coords)
            
            for i in range(int((self.n_followers - 1) / 2)):
                dN = (-0.866*offset*(i+1))-offset
                dE = (offset*(i+1))/2
                converted_offsets = self.drone_to_ned_conversion(dNorth=dN, dEast=dE, heading=self.data.euler_orientation.yaw)
                coords = self.ellipsoid_offset_location(latitude=self.data.global_position.latitude,
                                                        longitude=self.data.global_position.longitude,
                                                        dNorth=converted_offsets[0],
                                                        dEast=converted_offsets[1])
                follower_coordinates.append(coords)
                
                dE = (-1*offset*(i+1))/2
                converted_offsets = self.drone_to_ned_conversion(dNorth=dN, dEast=dE, heading=self.data.euler_orientation.yaw)
                coords = self.ellipsoid_offset_location(latitude=self.data.global_position.latitude,
                                                        longitude=self.data.global_position.longitude,
                                                        dNorth=converted_offsets[0],
                                                        dEast=converted_offsets[1])
                follower_coordinates.append(coords)
        
        return follower_coordinates
    
    def calculate_helical_formation_coordinates(self, offset=5):
        follower_coordinates = []
        angle_increment = 2 * math.pi / 3
        current_angle = 0
        
        for i in range(self.n_followers):
            dN = offset * math.cos(current_angle)
            dE = offset * math.sin(current_angle)
            
            converted_offsets = self.drone_to_ned_conversion(dNorth=dN, dEast=dE, heading=self.data.euler_orientation.yaw)
            
            new_coords = self.ellipsoid_offset_location(latitude=self.data.global_position.latitude,
                                                        longitude=self.data.global_position.longitude,
                                                        dNorth=converted_offsets[0],
                                                        dEast=converted_offsets[1])
            follower_coordinates.append(new_coords)
            
            current_angle += angle_increment
            
        return follower_coordinates