#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Empty.h"  
#include <thread>
#include <atomic>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    ROS_INFO("Gripper open posture has been sent.");

    posture.joint_names.resize(1);
    posture.joint_names[0] = "right_finger_bottom_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.95; // must < 0.96
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    ROS_INFO("Gripper close posture has been sent.");

    posture.joint_names.resize(1);
    posture.joint_names[0] = "right_finger_bottom_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.2; //-0.09;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void getObjectInfo(ros::NodeHandle& node_handle, const std::string& object_id, 
                   geometry_msgs::Pose& object_pose, std::vector<double>& object_dimensions)
{
    ros::ServiceClient planning_scene_client = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    if (planning_scene_client.call(srv)) {
        //ROS_INFO("Service call to get_planning_scene succeeded.");
        const auto& objects = srv.response.scene.world.collision_objects;
        for (const auto& object : objects) {
            if (object.id == object_id) {
                object_pose = object.pose;
                if (!object.primitives.empty()) {
                    object_dimensions = object.primitives[0].dimensions;
                    ROS_INFO("Object %s was found in the planning scene.", object_id.c_str());
                    ROS_INFO("Object position: [x: %f, y: %f, z: %f]", object_pose.position.x, object_pose.position.y, object_pose.position.z);
                    ROS_INFO("Object dimensions: [x: %f, y: %f, z: %f]", object_dimensions[0], object_dimensions[1], object_dimensions[2]);
                } else {
                    ROS_ERROR("Object %s has no primitives!", object_id.c_str());
                }
                return;
            }
        }
        ROS_ERROR("Object %s not found in the planning scene!", object_id.c_str());
    } else {
        ROS_ERROR("Failed to call service get_planning_scene");
    }
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, 
          ros::NodeHandle& node_handle)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    geometry_msgs::Pose object_pose;
    std::vector<double> object_dimensions;

    // 使用GetPlanningScene服务获取最新场景信息
    getObjectInfo(node_handle, "object", object_pose, object_dimensions);

    // 设置抓取姿态
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 4, 0, -tau / 4); // 固定轴欧拉角,此时夹爪正对x轴方向
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position = object_pose.position;

    // 设置预抓取接近方式
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = object_dimensions[0] / 2 + 0.05;
    grasps[0].pre_grasp_approach.desired_distance = object_dimensions[0] / 2 + 0.1;

    // 设置抓取后的退回方式
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // 设置抓取前的夹爪姿态
    openGripper(grasps[0].pre_grasp_posture);

    // 设置抓取时的夹爪姿态
    closedGripper(grasps[0].grasp_posture);

    // 设置支持面为 table1
    move_group.setSupportSurfaceName("table1");
    // 调用 pick 函数执行抓取
    ROS_INFO("Picking......");
    move_group.pick("object", grasps);
    ROS_INFO("Pick Finished.");
}


void place(moveit::planning_interface::MoveGroupInterface& move_group,
           ros::NodeHandle& node_handle)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    geometry_msgs::Pose object_pose;
    std::vector<double> object_dimensions;

    // 使用GetPlanningScene服务获取最新场景信息
    getObjectInfo(node_handle, "table2", object_pose, object_dimensions);

    // 设置放置位置的姿态
    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, tau / 4);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    place_location[0].place_pose.pose.position = object_pose.position;
    place_location[0].place_pose.pose.position.z += object_dimensions[2] / 2 + 0.08;

    // 设置放置前的接近姿态
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = object_dimensions[2] / 2 + 0.05;
    place_location[0].pre_place_approach.desired_distance = object_dimensions[2] / 2 + 0.1;

    // 设置放置后的撤退姿态
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // 设置放置后的夹爪姿态为打开
    openGripper(place_location[0].post_place_posture);

    // 设置支撑表面为table2
    move_group.setSupportSurfaceName("table2");
    // 调用place函数，将对象放置到指定的位置
    ROS_INFO("Placing......");
    move_group.place("object", place_location);
    ROS_INFO("Place Finished.");
}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
                         ros::NodeHandle& node_handle)
{
    // 创建环境
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // 第1个桌子
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.3;
    collision_objects[0].primitives[0].dimensions[1] = 0.3;
    collision_objects[0].primitives[0].dimensions[2] = 0.06;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[0].ADD;

    // 第2个桌子
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.3;
    collision_objects[1].primitives[0].dimensions[1] = 0.3;
    collision_objects[1].primitives[0].dimensions[2] = 0.06;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    collision_objects[1].operation = collision_objects[1].ADD;

    // 操作对象
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].id = "object";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.06;
    collision_objects[2].primitives[0].dimensions[1] = 0.06;
    collision_objects[2].primitives[0].dimensions[2] = 0.16;
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.11;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].operation = collision_objects[2].ADD;

    // 应用到规划场景
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface arm_group("arm");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

    arm_group.setPlannerId("RRTConnectkConfigDefault"); // 规划算法
    arm_group.setPlanningTime(10.0);
    arm_group.setNumPlanningAttempts(10);

    addCollisionObjects(planning_scene_interface, nh);

    pick(arm_group, nh);

    place(arm_group, nh);

    ros::waitForShutdown();
    return 0;
}
