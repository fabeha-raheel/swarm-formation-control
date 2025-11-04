#!/bin/bash

cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris2 --console -I1 --out=tcpin:0.0.0.0:8200 