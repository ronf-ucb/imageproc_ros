#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/ronf/Research/ros/groovy_workspace/Turtle/Turner25/build/catkin_generated', type 'exit' to leave"
  . "/home/ronf/Research/ros/groovy_workspace/Turtle/Turner25/build/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/ronf/Research/ros/groovy_workspace/Turtle/Turner25/build/catkin_generated'"
else
  . "/home/ronf/Research/ros/groovy_workspace/Turtle/Turner25/build/catkin_generated/setup_cached.sh"
  exec "$@"
fi
