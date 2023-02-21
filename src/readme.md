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

# TODO

- [ ] Fix Joint for gripper
- [x] Remove unused files
- [ ] Create group with dual robot in moveit
- [ ] create group with hand + robot
- [ ] Replace file name
