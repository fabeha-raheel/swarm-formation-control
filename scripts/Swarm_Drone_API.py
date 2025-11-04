#!/usr/bin/python3

import math
import rospy
import time

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.msg import GlobalPositionTarget, ParamValue, State, RCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSet, StreamRate

from Drone_Data import Drone_Data


class MAVROS_Drone():
    def __init__(self, ns=None) -> None:

        self.ns = ns
        self.data = Drone_Data()
        
    def init_node(self):
        rospy.init_node("MAVROS_Drone_Node")           
        
    def init_subscribers(self):
        if self.ns is not None:
            self.global_position_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)
            self.rel_alt_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/rel_alt',Float64, self.rel_alt_sub_cb)    
            self.state_subscriber = rospy.Subscriber(self.ns + '/mavros/state',State, self.state_sub_cb)
            self.rc_subscriber = rospy.Subscriber(self.ns + '/mavros/rc/in', RCIn, self.rc_sub_cb)        
        else:
            self.global_position_subscriber = rospy.Subscriber('/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber('/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)
            self.rel_alt_subscriber = rospy.Subscriber('/mavros/global_position/rel_alt',Float64, self.rel_alt_sub_cb) 
            self.state_subscriber = rospy.Subscriber('/mavros/state',State, self.state_sub_cb) 
            self.rc_subscriber = rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_sub_cb) 
            
    def init_publishers(self):
        if self.ns is not None:
            self.global_setpoint_publisher = rospy.Publisher(self.ns + '/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
        else:
            self.global_setpoint_publisher = rospy.Publisher('/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
            
    def check_GPS_fix(self):
        if self.data.global_position.gps_fix >= 0:
            return True
        else:
            return False
    
    def arm(self):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse

    def disarm(self):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse
            
            
    def set_mode(self, mode):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy(self.ns + '/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        
        return modeResponse
            
    def takeoff(self, altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        if self.ns is None:
            rospy.wait_for_service('/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
        else:
            rospy.wait_for_service(self.ns + '/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy(self.ns + '/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
            
        if takeoffResponse:
            self.takeoff_altitude = altitude
        else:
            self.takeoff_altitude = 0
            
        return takeoffResponse
    
    def set_stream_rate(self):
        if self.ns is None:
            rospy.wait_for_service('/mavros/set_stream_rate', timeout=3)
            try:
                streamService = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
                streamService(0, 10, 1)
            except rospy.ServiceException as e:
                print("Setting Stream Rate failed: %s" %e)
        else:
            rospy.wait_for_service(self.ns + '/mavros/set_stream_rate', timeout=3)
            try:
                streamService = rospy.ServiceProxy(self.ns + '/mavros/set_stream_rate', StreamRate)
                streamService(0, 10, 1)
            except rospy.ServiceException as e:
                print("Setting Stream Rate failed: %s" %e)

    def wait_for_guided(self):
        while self.data.header.mode != 'GUIDED':
            time.sleep(0.5)
    
    def goto_location(self, latitude, longitude, altitude, type_mask=4088, coordinate_frame=6):
        
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.type_mask=type_mask
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)
        
    def goto_location_heading(self, latitude, longitude, altitude, yaw, type_mask=4088, coordinate_frame=6):
        
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.yaw = yaw
        command.type_mask=GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ | GlobalPositionTarget.IGNORE_YAW_RATE
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)
    
    def set_param(self, param_name, param_value):
        if self.ns is None:
            rospy.wait_for_service('/mavros/param/set', timeout=3)
            try:
                param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
                
                value = ParamValue()
                value.integer = 0
                value.real = float(param_value)
                response = param_set(param_name, value)
                rospy.loginfo("{0} parameter set to {1}".format(param_name, param_value))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
        
        else:
            rospy.wait_for_service(self.ns + '/mavros/param/set', timeout=3)
            try:
                param_set = rospy.ServiceProxy(self.ns + '/mavros/param/set', ParamSet)
                value = ParamValue()
                value.integer = 0
                value.real = float(param_value)
                response = param_set(param_name, value)
                rospy.loginfo("{0} parameter set to {1}".format(param_name, param_value))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                
        return response
    
    def check_takeoff_complete(self):
        if abs(self.takeoff_altitude - self.data.local_position.z) <= 0.5:
            return True
        else:
            return False
        
    def check_land_complete(self):
        if self.data.local_position.z <= 0.5:
            return True
        else:
            return False
        
    def check_target_location_reached(self, location):
        pass
                 
    def global_sub_cb(self, mssg):
        self.data.global_position.gps_fix = mssg.status.status
        self.data.global_position.latitude = mssg.latitude
        self.data.global_position.longitude = mssg.longitude
        
    def local_sub_cb(self, mssg):
        self.data.local_position.x = mssg.pose.pose.position.x
        self.data.local_position.y = mssg.pose.pose.position.y
        self.data.local_position.z = mssg.pose.pose.position.z
        
    def hdg_sub_cb(self,mssg):
        self.data.euler_orientation.yaw = mssg.data

    def rel_alt_sub_cb(self, mssg):
        self.data.global_position.altitude = mssg.data
        
    def state_sub_cb(self, mssg):
        self.data.header.mode = mssg.mode

    def rc_sub_cb(self, mssg):
        self.data.rc = mssg.channels
        
    def offset_location(self, latitude, longitude, dNorth, dEast):
        earth_radius = 6378137.0 #Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*latitude/180))
        #New position in decimal degrees
        newlat = latitude + (dLat * 180/math.pi)
        newlon = longitude + (dLon * 180/math.pi)
        targetlocation = (newlat, newlon)
            
        return targetlocation
    
    def ellipsoid_offset_location(self, latitude, longitude, dNorth, dEast):
        # WGS84 Earth semi-major and semi-minor axes
        a = 6378137.0  # semi-major axis in meters
        b = 6356752.314245  # semi-minor axis in meters

        # Eccentricity squared
        e_squared = 1 - (b**2 / a**2)

        # Radius of curvature in the prime vertical
        N = a / math.sqrt(1 - e_squared * math.sin(math.radians(latitude))**2)

        # Latitude offset in radians
        dLat = dNorth / N

        # Longitude offset in radians
        dLon = dEast / (N * math.cos(math.radians(latitude)))

        # Update latitude and longitude in radians
        new_lat_radians = math.radians(latitude) + dLat
        new_lon_radians = math.radians(longitude) + dLon

        # Convert back to decimal degrees
        new_lat = math.degrees(new_lat_radians)
        new_lon = math.degrees(new_lon_radians)

        target_location = (new_lat, new_lon)
        
        return target_location
    
    def drone_to_ned_conversion(self, dNorth, dEast, heading):
        if heading >= 270:
            hdg = heading - 270
            North = dNorth*math.sin(math.radians(hdg)) + dEast*math.cos(math.radians(hdg))
            East = -1*dNorth*math.cos(math.radians(hdg)) + dEast*math.sin(math.radians(hdg))
        elif heading <= 90:
            North = dNorth*math.cos(math.radians(heading)) - dEast*math.sin(math.radians(heading))
            East = dNorth*math.sin(math.radians(heading)) + dEast*math.cos(math.radians(heading))
        elif heading<=180 and heading>90:
            hdg = heading - 90
            North = -1*dNorth*math.sin(math.radians(hdg)) - dEast*math.cos(math.radians(hdg))
            East = dNorth*math.cos(math.radians(hdg)) - dEast*math.sin(math.radians(hdg))
        else:
            hdg = heading - 180
            North = -1*dNorth*math.cos(math.radians(hdg)) + dEast*math.sin(math.radians(hdg))
            East = -1*dNorth*math.sin(math.radians(hdg)) - dEast*math.cos(math.radians(hdg))
        return (North, East)
    
    def ned_to_drone_conversion(self, dNorth, dEast, heading):
        North = dNorth*math.cos(math.radians(heading)) - dEast*math.sin(math.radians(heading))
        East = dNorth*math.sin(math.radians(heading)) + dEast*math.cos(math.radians(heading))
        return (North, East)