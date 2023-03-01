
#include "ros/ros.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"


visualization_msgs::Marker addTableVisualization();
moveit_msgs::CollisionObject addTableCO();
moveit_msgs::CollisionObject addSwitchgearCO();
moveit_msgs::CollisionObject addSideCO();
moveit_msgs::CollisionObject addBackCO();

int main(int argc, char** argv)
{  

    ros::init(argc, argv, "collision_object_publisher");
    ros::NodeHandle nh;


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    planning_scene_interface.applyCollisionObject(addTableCO());
    //planning_scene_interface.applyCollisionObject(addSideCO());
    //planning_scene_interface.applyCollisionObject(addBackCO());


    // rviz table visualizer
    ros::Publisher table_pub = nh.advertise<visualization_msgs::MarkerArray>("table_visualizer", 1);


    visualization_msgs::MarkerArray MarkerArray;
    visualization_msgs::Marker new_marker = addTableVisualization();
    MarkerArray.markers.push_back(new_marker);


    while(ros::ok() && ! ros::isShuttingDown())
    {
        for(int i=0; i<MarkerArray.markers.size(); ++i)
            MarkerArray.markers[i].header.stamp = ros::Time();
        
        table_pub.publish(MarkerArray);
        ros::spinOnce();
    }
    return 0;
}





moveit_msgs::CollisionObject addTableCO()
{
    moveit_msgs::CollisionObject robot_base;
    robot_base.id = "Robot_base";
    robot_base.header.frame_id = "world";

    robot_base.primitives.resize(1);
    robot_base.primitives[0].type = robot_base.primitives[0].BOX;
    robot_base.primitives[0].dimensions.resize(3);
    robot_base.primitives[0].dimensions[0] = 1;
    robot_base.primitives[0].dimensions[1] = 2;
    robot_base.primitives[0].dimensions[2] = 0.01;

    robot_base.primitive_poses.resize(1);
    robot_base.primitive_poses[0].position.x = 0.0;
    robot_base.primitive_poses[0].position.y = 0;
    robot_base.primitive_poses[0].position.z = -0.004;
    robot_base.primitive_poses[0].orientation.x = 0.0;
    robot_base.primitive_poses[0].orientation.y = 0.0;
    robot_base.primitive_poses[0].orientation.z = 0.0;
    robot_base.primitive_poses[0].orientation.w = 1.0;
    robot_base.operation = robot_base.ADD;

    return robot_base;
}

moveit_msgs::CollisionObject addSideCO()
{
    moveit_msgs::CollisionObject robot_base;
    robot_base.id = "robot_side_limit";
    robot_base.header.frame_id = "world";

    robot_base.primitives.resize(1);
    robot_base.primitives[0].type = robot_base.primitives[0].BOX;
    robot_base.primitives[0].dimensions.resize(3);
    robot_base.primitives[0].dimensions[0] = 2;
    robot_base.primitives[0].dimensions[1] = 0.01;
    robot_base.primitives[0].dimensions[2] = 1;

    robot_base.primitive_poses.resize(1);
    robot_base.primitive_poses[0].position.x = 0.5;
    robot_base.primitive_poses[0].position.y = -0.5;
    robot_base.primitive_poses[0].position.z = 0.5;
    robot_base.operation = robot_base.ADD;

    return robot_base;
}



moveit_msgs::CollisionObject addBackCO()
{
    moveit_msgs::CollisionObject robot_base;
    robot_base.id = "robot_back_limit";
    robot_base.header.frame_id = "world";

    robot_base.primitives.resize(1);
    robot_base.primitives[0].type = robot_base.primitives[0].BOX;
    robot_base.primitives[0].dimensions.resize(3);
    robot_base.primitives[0].dimensions[0] = 0.01;
    robot_base.primitives[0].dimensions[1] = 1;
    robot_base.primitives[0].dimensions[2] = 1;

    robot_base.primitive_poses.resize(1);
    robot_base.primitive_poses[0].position.x = -0.5;
    robot_base.primitive_poses[0].position.y = 0;
    robot_base.primitive_poses[0].position.z = 0.5;
    robot_base.operation = robot_base.ADD;

    return robot_base;
}


visualization_msgs::Marker addTableVisualization()
{
    visualization_msgs::Marker new_marker;
    new_marker.header.frame_id = "world";
    new_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    new_marker.action = visualization_msgs::Marker::ADD;
    new_marker.mesh_resource = "package://dual_ur_description/model/table.stl";

    new_marker.color.r = 160.0/255.0;
    new_marker.color.g = 82.0/255.0;
    new_marker.color.b = 45.0/255.0;
    new_marker.color.a = 1.0;
    new_marker.scale.x = 1.0;
    new_marker.scale.y = 1.0;
    new_marker.scale.z = 1.0;
    new_marker.pose.position.x = 0.0;
    new_marker.pose.position.y = 0.0;
    new_marker.pose.position.z = -0.33;
    new_marker.pose.orientation.x = 0.0;
    new_marker.pose.orientation.y = 0.0;
    new_marker.pose.orientation.z = 0.0;
    new_marker.pose.orientation.w = 1.0;
    return new_marker;
}