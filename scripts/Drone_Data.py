#!/usr/bin/python3

class Drone_Data():
    
    def __init__(self) -> None:
        self.header = Header()
        self.local_position = LocalPosition()
        self.global_position = GlobalPosition()
        self.euler_orientation = EulerOrientation()
        self.linear_velocity = LinearVelocity()
        self.angular_velocity = AngularVelocity()
        self.linear_acceleration = LinearAcceleration()
        self.rc = RadioChannels()
        

class Header:
    def __init__(self) -> None:
        self.name = 'no-name'
        self.id = 1
        self.mode = 'NONE'

class LocalPosition:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class GlobalPosition:
    def __init__(self, fix=-1, latitude=0, longitude=0, altitude=0):
        self.gps_fix = fix
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

class EulerOrientation:
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class LinearAcceleration:
    def __init__(self, ax=0, ay=0, az=0):
        self.ax = ax
        self.ay = ay
        self.az = az

class AngularVelocity:
    def __init__(self, wx=0, wy=0, wz=0):
        self.wx = wx
        self.wy = wy
        self.wz = wz

class LinearVelocity:
    def __init__(self, vx=0, vy=0, vz=0):
        self.vx = vx
        self.vy = vy
        self.vz = vz

class RadioChannels:
    def __init__(self):
        self.rc = []