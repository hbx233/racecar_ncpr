# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/hbx/ros_proj_ws/devel_isolated/relaxed_astar;/home/hbx/ros_proj_ws/devel_isolated/racecar_planning;/home/hbx/ros_proj_ws/devel_isolated/racecar_mapping;/home/hbx/ros_proj_ws/devel_isolated/racecar_localization;/home/hbx/ros_proj_ws/devel_isolated/racecar_gazebo;/home/hbx/ros_proj_ws/devel_isolated/racecar_description;/home/hbx/ros_proj_ws/devel_isolated/racecar_control;/home/hbx/ros_proj_ws/devel_isolated/racecar;/home/hbx/ros_proj_ws/devel_isolated/joy_twist;/home/hbx/ros_proj_ws/devel_isolated/cartographer_rviz;/home/hbx/ros_proj_ws/devel_isolated/cartographer_ros;/home/hbx/ros_proj_ws/devel_isolated/cartographer_ros_msgs;/home/hbx/ros_proj_ws/devel_isolated/ackermann_cmd_mux;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/hbx/ros_proj_ws/src/racecar_ncpr/build/devel/env.sh')

output_filename = '/home/hbx/ros_proj_ws/src/racecar_ncpr/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
