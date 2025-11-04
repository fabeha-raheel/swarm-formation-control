# Multi UAV Simulation using ArduPilot

## Clone the uav_swarms repository
Make sure that ROS Noetic is installed and catkin workspace has been set up.
```bash
cd ~/catkin_ws/src
git clone https://github.com/fabeha-raheel/uav_swarms.git
```

## (No need to follow these steps!!) Modifying Iris Drone Model for Multi-UAV Simulation

Copy the iris drone model files and rename the model directories to distinguish between multiple drones (drones can be numbered as 1, 2, 3, etc).

In each model directory, open the model.config file and modify the name of the model enclosed within the <name></name> tags. This will be name which will be used to search for the model.

In each model directoryy, open the model.sdf file and search for <fdm_port_in></fdm_port_in> and <fdm_port_out></fdm_port_out> tags. You can keep 9002/9003 for drone 1, 9012/9013 for drone 2, 9022/9023 for drone 3 etc.

## Creating Multi-UAV Ardupilot SITL Instances with Unique Parameters

In order to simulate drones with different parameters we will need to create our own custom frames and parameter files.

Open the file ```ardupilot/Tools/autotest/pysim/vehicleinfo.py``` and add the following lines under the SIM section.

```python
            # Added for Multi-UAV Simulation
            "gazebo-iris1": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-iris1.parm"],
            },
            "gazebo-iris2": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-iris2.parm"],
            },
            "gazebo-iris3": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-iris3.parm"],
            },
            ###
```

Save the file and exit.

Then, navigate into the ```ardupilot/Tools/autotest/default_params``` directory. Copy the param file for gazebo-iris and rename it as gazebo-iris1. Copy and rename again as gazebo-iris2 and gazebo-iris3.

Open each new param file. Add the following line at the end of gazebo-iris1.param:
```
SYSID_THISMAV 1
```
Add the same line at the end of the other new param files and modify the SYSID_THISMAV value to 2 for gazebo-iris2 and 3 for gazebo-iris3.

## Launching Multi-UAV ArduPilot SITL with Gazebo

Launch the Gazebo world:
```bash
roslaunch uav_swarms simulation.launch
```

In another terminal, navigate into the ardupilot/ArduCopter directory:
```bash
cd ~/ardupilot/ArduCopter
```
Launch the ArduPilot SITL instances for the drones in separate terminals. (Make sure that you are in the correct directory.) For the first drone:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris1 --console -I0
```
For second drone:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris2 --console -I1
```
For third drone:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris3 --console -I2
```

## Launching a MAVROS Instance for Each Drone
To utilize MAVROS, we need to connect a UDP MAVLINK stream. The different streams can be found using the following line which shows up on the terminal when we run the SITL (simvehicle.py) script.
```python
SIM_VEHICLE: "mavproxy.py" "--out" "127.0.0.1:14550" "--out" "127.0.0.1:14551" "--master" "tcp:127.0.0.1:5760" "--sitl" "127.0.0.1:5501" "--console"
```
In this line, we can see that the SITL is forwarding the MAVLINK data stream on two output ports: 14550 and 14551. We will connect MAVROS to port 14551.

The second drone will display the following:
```python
SIM_VEHICLE: "mavproxy.py" "--out" "127.0.0.1:14560" "--out" "127.0.0.1:14561" "--master" "tcp:127.0.0.1:5770" "--sitl" "127.0.0.1:5511" "--console"
```
Therefore, the port 14561 can be used to connect to drone2 with MAVROS.

Similarly, MAVROS instance for the third drone will connect to port 14571.

The different mavros instances can be launched using the following commands in different terminals:
```bash
roslaunch uav_swarms apm.launch fcu_url:=udp://127.0.0.1:14551@14555 mavros_ns:=/drone1 tgt_system:=1

roslaunch uav_swarms apm.launch fcu_url:=udp://127.0.0.1:14561@14565 mavros_ns:=/drone2 tgt_system:=2

roslaunch uav_swarms apm.launch fcu_url:=udp://127.0.0.1:14571@14575 mavros_ns:=/drone3 tgt_system:=3
```

## References

1. [Intelligent Quads - Swarming using ArduPilot](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md)
2. [Intelligent Quads - Drone Swarms using MAVROS](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/multi_mavros_drones.md)
