#!/usr/bin/python3

import time
import sys

from SwarmLeader_API import *

# RC Channel for selecting formation type:
formation_rc_channel = 9
# Min and Max ranges for RC for different formations - min value is inclusive and max value is exclusive
formation_rc_line_MinMax = [900, 1200]
formation_rc_flock_MinMax = [1250, 1650]
formation_rc_helical_MinMax = [1700, 2000]

takeoff_spacing = 3
formation_offset = 3
helical_offset = 5

leader = SwarmLeader(name='drone1', n_followers=2)

rospy.loginfo("Waiting for Leader to switch to Guided Mode")
leader.wait_for_guided()
leader.set_stream_rate()
rospy.loginfo("Leader's Mode is now GUIDED.")

rospy.loginfo("Registering Followers")
leader.initialize_followers()
leader.wait_for_GPS_Fix()       # check if all leader and followers have GPS Fix

rospy.loginfo("Setting Followers Mode to GUIDED")
for follower in leader.followers:
    follower_response = follower.set_mode(mode='GUIDED')
    
    if follower_response.mode_sent:
        rospy.loginfo("{} has been switched to GUIDED.".format(follower.data.header.name))
    else:
        rospy.loginfo("{} cannot be switched to GUIDED.".format(follower.data.header.name))
        
        rospy.loginfo("Removing {} from list of active followers.".format(follower.data.header.name))
        leader.followers.remove(follower)
        leader.n_followers = leader.n_followers - 1
    
rospy.loginfo("Arming Leader")
leader_response = leader.arm()

if leader_response.success:
    rospy.loginfo("Leader's Mode is now ARMED.")
else:
    rospy.logerr("Leader is not Armable. Aborting Mission...")
    sys.exit(1)
    
# Leader Take-off
leader_min_altitude = (takeoff_spacing*leader.n_followers)+takeoff_spacing       
leader_response = leader.takeoff(altitude=leader_min_altitude)

if leader_response.success:
    rospy.loginfo("Leader is Taking off.")
else:
    rospy.logerr("Leader failed to takeoff. Aborting Mission...")
    sys.exit(1)

rospy.loginfo("Arming Followers")
for follower in leader.followers:
    follower_response = follower.arm()
    
    if follower_response.success:
        rospy.loginfo("{} is ARMED.".format(follower.data.header.name))
    else:
        rospy.loginfo("{} cannot be ARMED.".format(follower.data.header.name))
        
        # rospy.loginfo("Removing {follower.data.header.name} from list of active followers.")
        # leader.followers.remove(follower)
        # leader.n_followers = leader.n_followers - 1

# Followers Take-off
for follower in leader.followers:
    follower_index = leader.followers.index(follower)
    follower_response = follower.takeoff(altitude=(takeoff_spacing*(leader.n_followers-(follower_index+1)))+takeoff_spacing)
    
    if follower_response.success:
        rospy.loginfo("{} is Taking off.".format(follower.data.header.name))
        
    else:
        rospy.loginfo("{} failed to takeoff.".format(follower.data.header.name))
    
        # rospy.loginfo("Removing {follower.data.header.name} from list of active followers.")
        # leader.followers.remove(follower)
        # leader.n_followers = leader.n_followers - 1
    
while not leader.check_takeoff_complete():
    # Waiting for leader to complete takeoff
    time.sleep(0.1)

for follower in leader.followers:
    while not follower.check_takeoff_complete():
        # Waiting for follower to complete takeoff
        time.sleep(0.1)
        
for follower in leader.followers:
        follower_index = leader.followers.index(follower)
        rospy.loginfo("Setting RTL_ALT param of {}.".format(follower.data.header.name))
        follower.set_param(param_name="RTL_ALT", param_value=(takeoff_spacing*(leader.n_followers-(follower_index+1)))+takeoff_spacing)

rospy.loginfo("Ready to execute Swarm Mission!")

while not rospy.is_shutdown(): 
    if leader.data.rc[formation_rc_channel-1] >= formation_rc_flock_MinMax[0] and leader.data.rc[formation_rc_channel-1] < formation_rc_flock_MinMax[1]:       
        follower_coordinates = leader.calculate_flock_formation_coordinates(offset=formation_offset)
    elif leader.data.rc[formation_rc_channel-1] >= formation_rc_helical_MinMax[0] and leader.data.rc[formation_rc_channel-1] < formation_rc_helical_MinMax[1]:
        follower_coordinates = leader.calculate_helical_formation_coordinates(offset=helical_offset)
    else:
        follower_coordinates = leader.calculate_line_formation_coordinates(offset=formation_offset)

    heading = leader.data.euler_orientation.yaw

    if leader.data.global_position.altitude >= leader_min_altitude:  
        for follower in leader.followers:
            follower_index = leader.followers.index(follower)
            # follower.goto_location_heading(latitude=follower_coordinates[follower_index][0], 
            #                             longitude=follower_coordinates[follower_index][1], 
            #                             altitude=(2*(leader.n_followers-(follower_index+1)))+2,
            #                             yaw=heading)
            follower.goto_location(latitude=follower_coordinates[follower_index][0], 
                                        longitude=follower_coordinates[follower_index][1], 
                                        altitude=leader.data.global_position.altitude - (takeoff_spacing*(follower_index+1)))
            
    else:
        for follower in leader.followers:
            follower_index = leader.followers.index(follower)
            # follower.goto_location_heading(latitude=follower_coordinates[follower_index][0], 
            #                             longitude=follower_coordinates[follower_index][1], 
            #                             altitude=(2*(leader.n_followers-(follower_index+1)))+2,
            #                             yaw=heading)
            follower.goto_location(latitude=follower_coordinates[follower_index][0], 
                                        longitude=follower_coordinates[follower_index][1], 
                                        altitude=(takeoff_spacing*(leader.n_followers-(follower_index+1)))+takeoff_spacing)
            
    if leader.data.header.mode == "LAND":
        rospy.loginfo("Landing followers...")
        for follower in leader.followers:
            follower_response = follower.set_mode(mode='LAND')
        break
    elif leader.data.header.mode == "RTL":
        rospy.loginfo("Return to Home...")
        for follower in leader.followers:
            follower_response = follower.set_mode(mode='RTL')
        break
       
for follower in leader.followers:
    while not follower.check_land_complete():
        # print("...")
        time.sleep(0.1)
rospy.loginfo("All Followers have landed.")

while not leader.check_land_complete():
    # print("...")
    time.sleep(0.1)
    
time.sleep(2)
    
rospy.loginfo("Leader has successfully landed.")
    
rospy.loginfo("Mission Successfull.")