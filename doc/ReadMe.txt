R. Fearing
March 6, 2013

ROS nodes for position control of robot using optitrack position
feedback, and velocity commands sent to robot. Telemetry data is
recorded from robot at 10 Hz.

edit control.launch to use:
     optitrack_sim2.py for simulated position at 100 Hz
     optitrack_sim.py to use optitrack or position from keyboard

1. To run everything
   roslaunch Turner25 turnertest.launch

2. sendcommand.launch  publishes on RunTime to enable robot running.
   rosnode kill RunTimeSet
   will cause robot to stop running (but still recording telemetry data)

3. {kpx, kpy, kd} in controller.py set velocity per distance gain
   {MAX_VEL, MAX_TURN} in controller.py set max velocities 

4. Motion is set to 4 leg strides in run_robot_class.run().

5. leg cycle is set to 100 ms in robot_init.py

6. to create bagfile of most data
   rosbag record -a 

7. Convert bagfile to textfile (./Data/telemdata.txt)
   python turnerbag2data bagfile.bag

8. Convert .txt to .dat for plotting, and plot using xmgrace
   ./plot-data.batch
