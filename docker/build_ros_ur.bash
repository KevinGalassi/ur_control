#!/bin/bash

# WINDOWS: BASH file_name.bash
# UNIX: sh file_name.bash
docker build -f dockerfile_ros_ur -t ros_ur .
