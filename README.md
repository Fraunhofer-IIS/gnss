gnss
====

GNSS stack for ROS

Tools for working with Global Navigation Satellite Systems.


Usage
-----

The node `enu_to_pose` in `nodes/enu_to_pose.py` needs a network connection to
a rtklib instance. rtklib is built in this package with the Makefile (use
`rosmake`). rtklib then should supply the solution output on the network
connection configured in the `enu_to_pose`-node. Examples for rtklib with
solution output on the default network port and input of data from another
stream are included as examples.

The launch file `rtklib.launch` starts an rtklib instance as well as the
`enu_to_pose`-node. For testing purposes it is helpful to start rtklib
manually with the rtknavi-GUI or manually with the simple shell wrapper script
`start_rtklib.sh`.

Example step-by-step instructions:


* Add package to your ROS workspace with `rosinstall`
* Build rtklib with `rosmake`
* Supply a pre-recorded skytraq measurements file to the port specified in
  `enu_single.conf`, e.g. `cat <skytraq_measurements_file> | nc -l -p 10000`
* Start rtklib and `enu_to_pose`-node with `roslaunch rtklib.launch`
* Now, the rostopic `/enu` should show ENU pose informations. If the quality
  of the provided GNSS signal is sufficient, coordinate transforms are
  published to the ROS system


Advanced example procedure for debugging including intermediate testing steps:

* Provide GNSS measurements (see above)
* Start rtklib with `./start_rtklib.sh enu_single.conf` using these
  measurements
* Test rtklib solution by connecting to the output port manually and checking
  the solution for a valid position `nc localhost 3333`
* Ensure a roscore process is running: Call `roscore`
* Start the node with `rosrun rtklib enu_to_pose.py`


Example output
--------------

rtklib processes the GNSS measurements into position solutions in the ENU
coordinate system and provides these informations along with quality
information and other data on an output port, if configured, as can be seen in
the example configuration files. Each line should look like the following
example:

    2012/08/23 12:49:30.402    863028.7989   4720911.2174  -2194093.2995   5   9 3.0176   5.8136   3.4867   2.6834   3.1125   1.7950   0.00    0.0

The `enu_to_pose` node provides coordinate transforms and prints log messages
to the console. The position from rtklib is provided in the rostopic `/enu`
and looks like this:

    header:
      seq: 565
      stamp:
        secs: 1376397269
        nsecs: 369039058
      frame_id: /map
    pose:
      position:
        x: 863027.3836
        y: 4720911.2261
        z: -2194086.6277
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
