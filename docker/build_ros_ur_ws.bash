#!/bin/bash

# WINDOWS: BASH file_name.bash
# UNIX: sh file_name.bash

docker build --no-cache -f dockerfile_ros_ur_ws -t ros_ur_ws .
