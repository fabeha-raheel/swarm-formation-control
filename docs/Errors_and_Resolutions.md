### sim_vehicle.py: command not found
Try the following:
```bash
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --map --console
```

Or, try rebooting the PC and run the following commands:
```bash
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

### [Errno 2] No such file or directory: 'mavproxy.py'
(Encountered while running ArduPilot SITL / simvehicle.py script)

Resolution: Try rebooting the PC and try again. If error persists, delete the ardupilot directory from home and re-install the ArduPilot SITL package.
