# vive_ros2
Code for using the HTC vive tracking system with ROS2. Currently, this package only 
supports the HTC Vive tracker although tracking other devices like the tracking references (base stations), 
controllers, and headset should not require much code modification. Contributions are welcome. 

This package allows for maximum flexibility in terms of where the HTC drivers are run
since the package uses an independent server client architecture using python socket library.
For example, server can be run on a more powerful Windows machine for best driver support while
the client can be run on the robot or target device. The server and client can also
be run on the same device if desired. 

## Getting Started
### Running the Python Server
The best way to get started is to install conda and run `conda env create -f env.yaml` which 
will create a conda environment with the correct python version and libraries. You can then
activate the environment with `conda activate pythonvr`. Start the server by running `vive_tracker_server.py` in 
the `vive_server` folder. Make sure to have `SteamVR` installed and running with the headset plugged in.

Once you start the server you should see something like the following printed to the screen indicating which VR devices 
are detected and what the ip address and port (IP:PORT) of the server is. This information will come in handy when 
setting up the client.

```
13:49:17|ViveTrackerServer|INFO|Starting server at 192.168.50.171:8000
13:49:17|ViveTrackerServer|INFO|Connected VR devices: 
###########
Found 4 Tracking References
  tracking_reference_1 (LHB-59F8D726, Mode Valve SR Imp, Valve SR Imp)
  tracking_reference_2 (LHB-AA3BEC72, Mode Valve SR Imp, Valve SR Imp)
  tracking_reference_3 (LHB-FFCE0AD4, Mode Valve SR Imp, Valve SR Imp)
  tracking_reference_4 (LHB-91047ECC, Mode Valve SR Imp, Valve SR Imp)
Found 1 HMD
  hmd_1 (LHR-8280F84D, VIVE_Pro MV)
Found 1 Controller
  controller_1 (LHR-4F3DC6EA, VIVE Controller Pro MV)
Found 1 Tracker
  tracker_1 (LHR-55804C5D, VIVE Tracker Pro MV)
###########
```

### Running the ROS2 Client
In order to run the ROS2 node first run `sudo python3 setup.py install` to install the
package with the object type for the messages transmitted by the server. You may also have
to install some packages with pip such as pydantic if it is not already installed `pip3 install pydantic`.

You can then run the node by running `ros2 run vive_ros2 vive_tracker_node.py --ros-args -p host_ip:=******** 
-p host_port:=****`. You should then be able to run `ros2 topic echo /tracker/odom` to verify 
the node is working.
