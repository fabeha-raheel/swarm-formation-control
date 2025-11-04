#!/usr/bin/python3

import time
import sys

from SwarmLeader_API import *

takeoff_spacing = 3
formation_offset = 3

leader = SwarmLeader(name='drone1', n_followers=2)

rospy.loginfo("Registering Followers")
leader.initialize_followers()
leader.wait_for_GPS_Fix()       # check if all leader and followers have GPS Fix

rospy.loginfo("Setting Stream Rate")
leader.set_stream_rate()
time.sleep(0.5)

for follower in leader.followers:
    follower.set_stream_rate()
time.sleep(0.5)

rospy.loginfo("Setting Leader's Mode to GUIDED")
leader_response = leader.set_mode(mode='GUIDED')

if leader_response.mode_sent:
    rospy.loginfo("Leader's Mode is now GUIDED.")
else:
    rospy.logerr("Leader's Mode cannot be switched to GUIDED. Aborting Mission...")
    sys.exit(1)

rospy.loginfo("Setting Followers Mode to GUIDED")
for follower in leader.followers:
    follower_response = follower.set_mode(mode='GUIDED')
    
    if follower_response.mode_sent:
        rospy.loginfo("{follower.data.header.name} has been switched to GUIDED.")
    else:
        rospy.loginfo("{follower.data.header.name} cannot be switched to GUIDED.")
        
        rospy.loginfo("Removing {follower.data.header.name} from list of active followers.")
        leader.followers.remove(follower)
        leader.n_followers = leader.n_followers - 1
    
rospy.loginfo("Arming Leader")
leader_response = leader.arm()

if leader_response.success:
    rospy.loginfo("Leader's Mode is now ARMED.")
else:
    rospy.logerr("Leader is not Armable. Aborting Mission...")
    sys.exit(1)

rospy.loginfo("Arming Followers")
for follower in leader.followers:
    follower_response = follower.arm()
    
    if follower_response.success:
        rospy.loginfo("{follower.data.header.name} is ARMED.")
    else:
        rospy.loginfo("{follower.data.header.name} cannot be ARMED.")
        
        # rospy.loginfo("Removing {follower.data.header.name} from list of active followers.")
        # leader.followers.remove(follower)
        # leader.n_followers = leader.n_followers - 1
    
# Leader Take-off
target_altitude = (takeoff_spacing*leader.n_followers)+takeoff_spacing          # 2m spacing between each drone
leader_response = leader.takeoff(altitude=target_altitude)

if leader_response.success:
    rospy.loginfo("Leader is Taking off.")
else:
    rospy.logerr("Leader failed to takeoff. Aborting Mission...")
    sys.exit(1)

# Followers Take-off
for follower in leader.followers:
    follower_index = leader.followers.index(follower)
    follower_response = follower.takeoff(altitude=(takeoff_spacing*(leader.n_followers-(follower_index+1)))+takeoff_spacing)
    
    if follower_response.success:
        rospy.loginfo("{follower.data.header.name} is Taking off.")
    else:
        rospy.loginfo("{follower.data.header.name} failed to takeoff.")
        
        # rospy.loginfo("Removing {follower.data.header.name} from list of active followers.")
        # leader.followers.remove(follower)
        # leader.n_followers = leader.n_followers - 1
        
while not leader.check_takeoff_complete():
    # Waiting for leader to complete takeoff
    # print("...")
    time.sleep(0.1)
    
for follower in leader.followers:
    while not follower.check_takeoff_complete():
        # Waiting for follower to complete takeoff
        # print("...")
        time.sleep(0.1)
    
# Hover for few seconds
rospy.loginfo("Takeoff Complete. Hovering...")
time.sleep(5)

# Land all drones
rospy.loginfo("Landing followers...")
for follower in leader.followers:
    follower_response = follower.set_mode(mode='LAND')
    
rospy.loginfo("Landing Leader...")
leader.set_mode(mode="LAND")
    
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