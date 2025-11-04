#!/bin/bash

cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris3 --console -I2 --out=tcpin:0.0.0.0:8300 