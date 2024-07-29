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
    ROS_INFO("Opening gripper...");

    posture.joint_names.resize(1);
    posture.joint_names[0] = "right_finger_bottom_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.96;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    ROS_INFO("Closing gripper...");

    posture.joint_names.resize(1);
    posture.joint_names[0] = "right_finger_bottom_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = -0.09;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // 设置抓取姿态
    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY (-tau / 4, 0, -tau / 4);     //(-tau / 4, -tau / 8, -tau / 4);固定轴欧拉角
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.5; //0.415+0.10;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;
        
    // 设置预抓取接近方式
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // 设置抓取后的退回方式
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    /* Direction is set as positive z axis */
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
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group)
{
    // 创建一个存储放置位置的向量，大小为1
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // 设置放置位置的姿态
    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, tau / 4);  // 绕Z轴旋转四分之一圈
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // 对于放置位置，将值设置为对象中心的精确位置
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.5;
    place_location[0].place_pose.pose.position.z = 0.5;

    // 设置放置前的接近姿态
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    // 方向设置为负Z轴方向
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // 设置放置后的撤退姿态
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    // 方向设置为负Y轴方向
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // 设置放置后的夹爪姿态为打开
    openGripper(place_location[0].post_place_posture);

    // 设置支撑表面为table2
    move_group.setSupportSurfaceName("table2");
    // 调用place函数，将对象放置到指定的位置
    move_group.place("object", place_location);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // 创建环境
  // 向量保存3个碰撞对象
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // 第1个桌子
  collision_objects[0].id = "table1"; // ID
  collision_objects[0].header.frame_id = "base_link"; // 碰撞对象参考坐标系

  collision_objects[0].primitives.resize(1); // 形状
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX; // 盒子
  collision_objects[0].primitives[0].dimensions.resize(3); // 定义形状的尺寸数组
  collision_objects[0].primitives[0].dimensions[0] = 0.2; // 设置盒子的长
  collision_objects[0].primitives[0].dimensions[1] = 0.4; // 设置盒子的宽
  collision_objects[0].primitives[0].dimensions[2] = 0.4; // 设置盒子的高

  collision_objects[0].primitive_poses.resize(1); // 姿态
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0; // 无旋转

  collision_objects[0].operation = collision_objects[0].ADD;

  // 第2个桌子
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "base_link";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;


  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  collision_objects[1].operation = collision_objects[1].ADD;

  // 操作对象
  collision_objects[2].header.frame_id = "base_link";
  collision_objects[2].id = "object";

  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  collision_objects[2].operation = collision_objects[2].ADD;

  //应用到规划场景
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
    arm_group.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    // 创建 JointTrajectory 对象
    trajectory_msgs::JointTrajectory gripper_posture;

    pick(arm_group);

    place(arm_group);

    ros::waitForShutdown();
    return 0;
}