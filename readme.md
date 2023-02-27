# Real Robot #



Activate moveit for single arm
```
roslaunch ur_handler ur5e_bringup.launch 
```


Dual Robot
```
roslaunch dual_ur_robot dual_robot_startup.launch 
```
```
roslaunch dual_ur_moveit_config start_dual_ur_moveit.launch 
```




# Simulation


gazebo
```
roslaunch dual_ur_gazebo dual_ur_gazebo.launch 
```

for moveit 
```
roslaunch dual_ur_moveit_config start_sim_dual_ur_moveit.launch 
```

if mesh are not visiible :
```
export LC_NUMERIC="en_US.UTF-8"
```




# TODO

- [x] Fix Joint for gripper
- [x] Remove unused files
- [x] Create group with dual robot in moveit
- [x] create group with hand + robot
- [x] Replace file name
