#!/usr/bin/python3

from Swarm_Drone_API import MAVROS_Drone

class SwarmFollower(MAVROS_Drone):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        pass
        
        
        
if __name__ == '__main__':
    
    leader = SwarmFollower()