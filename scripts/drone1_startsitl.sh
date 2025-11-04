#!/bin/bash

cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris1 --console -I0 --out=tcpin:0.0.0.0:8100 