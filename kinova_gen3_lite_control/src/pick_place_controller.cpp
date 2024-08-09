#include <cmath>
#include <memory>
#include <thread>
#include <algorithm>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kinova_gen3_lite_control/AddRemoveCollisionObject.h>
#include <kinova_gen3_lite_control/PrintCollisionObjects.h>
#include <kinova_gen3_lite_control/MoveArmToPose.h>
#include <kinova_gen3_lite_control/Pick.h>
#include <kinova_gen3_lite_control/Place.h>
#include <kinova_gen3_lite_control/DetachAndRemoveObject.h>
#include "kinova_gen3_lite_control/GripperValue.h"
#include "kinova_gen3_lite_control/ManagePose.h"
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// 全局变量声明
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group;
std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
std::map<std::string, geometry_msgs::Pose> saved_poses;
std::string package_path = ros::package::getPath("kinova_gen3_lite_control");
std::string pose_file = package_path + "/config/pose_file.yaml";

// 函数声明
void openGripper(trajectory_msgs::JointTrajectory& posture);
void closedGripper(trajectory_msgs::JointTrajectory& posture, double position);
void getObjectInfo(ros::NodeHandle& node_handle, const std::string& object_id, 
                   geometry_msgs::Pose& object_pose, std::vector<double>& object_dimensions);
tf2::Vector3 calculateDirectionVector(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose);
double calculateCustomYawFromDirection(const tf2::Vector3& direction);
tf2::Quaternion calculateOrientation(double yaw);
tf2::Quaternion getRelativeOrientation(const tf2::Quaternion& current_orientation,
                                      const tf2::Quaternion& target_orientation);
void getRelativeEulerAngles(const tf2::Quaternion& relative_orientation,
                            double& roll, double& pitch, double& yaw);
void pick(moveit::planning_interface::MoveGroupInterface& move_group, 
          ros::NodeHandle& node_handle);
void place(moveit::planning_interface::MoveGroupInterface& move_group,
           ros::NodeHandle& node_handle);
bool pickCallback(kinova_gen3_lite_control::Pick::Request &req,
                  kinova_gen3_lite_control::Pick::Response &res);
bool placeCallback(kinova_gen3_lite_control::Place::Request &req,
                   kinova_gen3_lite_control::Place::Response &res);
bool detachAndRemoveObjectCallback(kinova_gen3_lite_control::DetachAndRemoveObject::Request &req,
                                   kinova_gen3_lite_control::DetachAndRemoveObject::Response &res);
void addCollisionObjects(std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_scene_interface, 
                         ros::NodeHandle& node_handle);
bool addRemoveCollisionObjectCallback(kinova_gen3_lite_control::AddRemoveCollisionObject::Request& req,
                                      kinova_gen3_lite_control::AddRemoveCollisionObject::Response& res);
bool printCollisionObjectsCallback(kinova_gen3_lite_control::PrintCollisionObjects::Request& req,
                                   kinova_gen3_lite_control::PrintCollisionObjects::Response& res);
bool moveArmToPoseCallback(kinova_gen3_lite_control::MoveArmToPose::Request &req,
                           kinova_gen3_lite_control::MoveArmToPose::Response &res);
bool sendGripperCommand(ros::NodeHandle n, double value);
bool gripper_control_callback(kinova_gen3_lite_control::GripperValue::Request& req,
                              kinova_gen3_lite_control::GripperValue::Response& res);
bool manage_pose_callback(kinova_gen3_lite_control::ManagePose::Request& req, kinova_gen3_lite_control::ManagePose::Response& res);
void savePosesToFile(const std::string& filename) ;
void loadPosesFromFile(const std::string& filename);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    // 初始化全局变量
    arm_group = std::unique_ptr<moveit::planning_interface::MoveGroupInterface>(new moveit::planning_interface::MoveGroupInterface("arm"));
    gripper_group = std::unique_ptr<moveit::planning_interface::MoveGroupInterface>(new moveit::planning_interface::MoveGroupInterface("gripper"));
    planning_scene_interface = std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>(new moveit::planning_interface::PlanningSceneInterface());

    arm_group->setPlannerId("RRTConnectkConfigDefault");
    arm_group->setPlanningTime(10.0);
    arm_group->setNumPlanningAttempts(10);

    // 加载之前保存的姿态数据
    loadPosesFromFile(pose_file);

    // 注册服务
    ros::ServiceServer add_remove_collision_object_service = nh.advertiseService(
        "add_remove_collision_object", addRemoveCollisionObjectCallback);
    ROS_INFO("Add/Remove Collision Object service ready.");

    ros::ServiceServer print_collision_objects_service = nh.advertiseService(
        "print_collision_objects", printCollisionObjectsCallback);
    ROS_INFO("Print Collision Objects service ready.");

    ros::ServiceServer move_arm_to_pose_service = nh.advertiseService(
        "move_arm_to_pose", moveArmToPoseCallback);
    ROS_INFO("Move Arm To Pose service ready.");

    ros::ServiceServer pick_service = nh.advertiseService("pick", pickCallback);
    ROS_INFO("Pick service ready.");

    ros::ServiceServer place_service = nh.advertiseService("place", placeCallback);
    ROS_INFO("Place service ready.");

    ros::ServiceServer detach_remove_service = nh.advertiseService("detach_and_remove_object", detachAndRemoveObjectCallback);
    ROS_INFO("Detach and Remove Object service ready.");

    ros::ServiceServer gripper_control_service = nh.advertiseService("gripper_control", gripper_control_callback);
    ROS_INFO("Gripper control service ready.");

    ros::ServiceServer manage_pose_service = nh.advertiseService("manage_pose", manage_pose_callback);
    ROS_INFO("Manage pose service ready.");

    //添加默认场景中的物体
    addCollisionObjects(planning_scene_interface, nh);

    // pick(*arm_group, nh);

    // place(*arm_group, nh);

    ros::waitForShutdown();
    return 0;
}

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(1);
    posture.joint_names[0] = "right_finger_bottom_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.95; // must < 0.96
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture, double position)
{
    posture.joint_names.resize(1);
    posture.joint_names[0] = "right_finger_bottom_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = position;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void getObjectInfo(ros::NodeHandle& node_handle, const std::string& object_id, 
                   geometry_msgs::Pose& object_pose, std::vector<double>& object_dimensions)
{
    // 使用PlanningSceneInterface获取附着物体
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 先检查附着的物体
    auto attached_objects = planning_scene_interface.getAttachedObjects({object_id});
    if (!attached_objects.empty()) {
        const auto& attached_object = attached_objects[object_id];
        object_pose = attached_object.object.pose;
        if (!attached_object.object.primitives.empty()) {
            object_dimensions = attached_object.object.primitives[0].dimensions;
            ROS_INFO("Attached object %s found on the robot.", object_id.c_str());
            return;
        } else {
            ROS_ERROR("Attached object %s has no primitives!", object_id.c_str());
            return;
        }
    }

    // 如果附着物体未找到，则尝试在场景中查找
    ros::ServiceClient planning_scene_client = node_handle.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    if (planning_scene_client.call(srv)) {
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

tf2::Vector3 calculateDirectionVector(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
{
    // 将起点和终点转换为tf2::Vector3
    tf2::Vector3 start(start_pose.position.x, start_pose.position.y, 0.0);
    tf2::Vector3 end(end_pose.position.x, end_pose.position.y, 0.0);
    // 计算方向向量
    tf2::Vector3 direction = end - start;
    direction.normalize();

    // 打印起点、终点和方向向量
    ROS_INFO("Start Position: [x: %f, y: %f, z: %f]", start.x(), start.y(), start.z());
    ROS_INFO("End Position: [x: %f, y: %f, z: %f]", end.x(), end.y(), end.z());
    ROS_INFO("Direction Vector (xoy plane): [x: %f, y: %f, z: %f]", direction.x(), direction.y(), direction.z());

    return direction;
}
double calculateCustomYawFromDirection(const tf2::Vector3& direction)
{
    double x = direction.x();
    double y = direction.y();

    // 计算从 y 轴正方向到方向向量的角度，逆时针为正
    double custom_yaw = atan2(-x, y);

    // 打印计算出的自定义 yaw 角度
    ROS_INFO("Calculated Custom Yaw (radians): %f", custom_yaw);

    return custom_yaw;
}

tf2::Quaternion calculateOrientation(double yaw)
{
    // 创建四元数表示夹爪的朝向
    tf2::Quaternion orientation;
    orientation.setRPY(-tau / 4, 0, yaw); // roll = -pai/2, pitch = 0, yaw = calculated yaw

    // 打印计算出的四元数
    ROS_INFO("Calculated Gripper Quaternion: [x: %f, y: %f, z: %f, w: %f]", orientation.x(), orientation.y(), orientation.z(), orientation.w());

    return orientation;
}

tf2::Quaternion getRelativeOrientation(const tf2::Quaternion& current_orientation,
                                      const tf2::Quaternion& target_orientation)
{
    // 计算从当前姿态到目标姿态的相对旋转
    tf2::Quaternion relative_orientation = target_orientation * current_orientation.inverse();

    // 打印相对姿态的四元数
    ROS_INFO("Relative Quaternion: [x: %f, y: %f, z: %f, w: %f]", 
            relative_orientation.x(), relative_orientation.y(),
            relative_orientation.z(), relative_orientation.w());

    return relative_orientation;
}

void getRelativeEulerAngles(const tf2::Quaternion& relative_orientation,
                            double& roll, double& pitch, double& yaw)
{
    // 从相对旋转的四元数中提取欧拉角
    tf2::Matrix3x3(relative_orientation).getRPY(roll, pitch, yaw);

    // 打印相对姿态的欧拉角
    ROS_INFO("Relative Euler Angles (radians) - Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, 
          ros::NodeHandle& node_handle,
          const std::string& object_name,
          const std::string& support_surface_name,
          double input_gripper_position)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    geometry_msgs::Pose object_pose, origin_pose;
    std::vector<double> object_dimensions;

    origin_pose.position.x = 0.0;
    origin_pose.position.y = 0.0;
    origin_pose.position.z = 0.0;

    // 使用GetPlanningScene服务获取最新场景信息
    getObjectInfo(node_handle, object_name, object_pose, object_dimensions);

    if (object_dimensions.empty()) {
        throw std::runtime_error("Failed to get object dimensions.");
    }

    // 当前姿态的四元数（假设为初始姿态）
    tf2::Quaternion current_orientation;
    current_orientation.setRPY(0, 0, 0); // 初始姿态为零姿态

    // 计算接近方向
    tf2::Vector3 approach_direction = calculateDirectionVector(origin_pose, object_pose);
    double yaw = calculateCustomYawFromDirection(approach_direction);
    tf2::Quaternion grasp_orientation = calculateOrientation(yaw);

    // 计算目标姿态与当前姿态的相对旋转
    tf2::Quaternion relative_orientation = getRelativeOrientation(current_orientation, grasp_orientation);
    
    // 提取相对旋转的欧拉角
    double roll, pitch, yaw_relative;
    getRelativeEulerAngles(relative_orientation, roll, pitch, yaw_relative);

    // 设置抓取姿态
    grasps[0].grasp_pose.header.frame_id = "base_link";
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(grasp_orientation);
    grasps[0].grasp_pose.pose.position = object_pose.position;

    // 设置预抓取接近方式
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps[0].pre_grasp_approach.direction.vector.x = approach_direction.x();
    grasps[0].pre_grasp_approach.direction.vector.y = approach_direction.y();
    grasps[0].pre_grasp_approach.direction.vector.z = 0.0; // z = 0
    grasps[0].pre_grasp_approach.min_distance = object_dimensions[0] / 2 + 0.05;
    grasps[0].pre_grasp_approach.desired_distance = object_dimensions[0] / 2 + 0.1;

    // 修改抓取后的退回方式为z轴正方向
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps[0].post_grasp_retreat.direction.vector.x = 0.0;
    grasps[0].post_grasp_retreat.direction.vector.y = 0.0;
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0; // z = 1
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // 设置抓取前的夹爪姿态
    openGripper(grasps[0].pre_grasp_posture);

    // 计算或使用传入的夹爪位置
    double gripper_position = std::max(object_dimensions[0] * 5 - 0.06, 0.0);  // 默认计算值
    if (input_gripper_position >= 0.0 && input_gripper_position <= 1.0) {
        gripper_position = input_gripper_position * 0.95;  // 将0~1映射到0~0.95
    }

    // 设置抓取时的夹爪姿态
    closedGripper(grasps[0].grasp_posture, gripper_position);

    // 设置支持面为传入的值
    move_group.setSupportSurfaceName(support_surface_name);

    // 调用 pick 函数执行抓取
    ROS_INFO("Picking object: %s on surface: %s with gripper position: %f", object_name.c_str(), support_surface_name.c_str(), gripper_position);
    move_group.pick(object_name, grasps);
    ROS_INFO("Pick Finished.^-^");
}

void place(moveit::planning_interface::MoveGroupInterface& move_group,
           ros::NodeHandle& node_handle,
           const std::string& object_name,
           const std::string& support_surface_name)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    geometry_msgs::Pose object_pose, target_pose, origin_pose;
    std::vector<double> object_dimensions, support_surface_dimensions;

    origin_pose.position.x = 0.0;
    origin_pose.position.y = 0.0;
    origin_pose.position.z = 0.0;

    // 使用GetPlanningScene服务获取支持面的信息
    getObjectInfo(node_handle, support_surface_name, target_pose, support_surface_dimensions);
    if (support_surface_dimensions.empty()) {
        throw std::runtime_error("Failed to get support surface dimensions.");
    }

    // 使用GetPlanningScene服务获取物体的信息
    getObjectInfo(node_handle, object_name, object_pose, object_dimensions);
    if (object_dimensions.empty()) {
        throw std::runtime_error("Failed to get object dimensions.");
    }

    // 获取当前机械臂的姿态
    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
    tf2::Quaternion current_orientation;
    tf2::fromMsg(current_pose.orientation, current_orientation);

    // 计算接近方向
    tf2::Vector3 approach_direction = calculateDirectionVector(origin_pose, target_pose);
    double yaw = calculateCustomYawFromDirection(approach_direction);
    tf2::Quaternion place_orientation = calculateOrientation(yaw);

    // 计算目标姿态与当前姿态的相对旋转
    tf2::Quaternion relative_orientation = getRelativeOrientation(current_orientation, place_orientation);
    
    // 提取相对旋转的欧拉角
    double roll, pitch, yaw_relative;
    getRelativeEulerAngles(relative_orientation, roll, pitch, yaw_relative);

    // 设置放置位置的姿态
    place_location[0].place_pose.header.frame_id = "base_link";
    place_location[0].place_pose.pose.orientation = tf2::toMsg(relative_orientation);
    place_location[0].place_pose.pose.position = target_pose.position;
    place_location[0].place_pose.pose.position.z += support_surface_dimensions[2] / 2 + object_dimensions[2] / 2;

    // 设置放置前的接近姿态（z轴负方向）
    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.x = 0.0;
    place_location[0].pre_place_approach.direction.vector.y = 0.0;
    place_location[0].pre_place_approach.direction.vector.z = -1.0; // z = -1
    place_location[0].pre_place_approach.min_distance = object_dimensions[2] / 2 + 0.05;
    place_location[0].pre_place_approach.desired_distance = object_dimensions[2] / 2 + 0.1;

    // 设置放置后的撤退姿态
    tf2::Vector3 retreat_direction = -approach_direction;
    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.x = retreat_direction.x();
    place_location[0].post_place_retreat.direction.vector.y = retreat_direction.y();
    place_location[0].post_place_retreat.direction.vector.z = 0.0; // z = 0
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // 设置放置后的夹爪姿态为打开
    openGripper(place_location[0].post_place_posture);

    // 设置支持面为传入的值
    move_group.setSupportSurfaceName(support_surface_name);

    // 调用 place 函数将对象放置到指定的位置
    ROS_INFO("Placing object: %s on surface: %s", object_name.c_str(), support_surface_name.c_str());
    move_group.place(object_name, place_location);
    ROS_INFO("Place Finished.^-^");
}

bool pickCallback(kinova_gen3_lite_control::Pick::Request &req,
                  kinova_gen3_lite_control::Pick::Response &res)
{
    ros::NodeHandle nh;

    // 检查物体是否存在于场景中
    auto world_objects = planning_scene_interface->getObjects({req.object_name});
    if (world_objects.find(req.object_name) == world_objects.end()) {
        res.success = false;
        res.message = "Object " + req.object_name + " does not exist in the planning scene.";
        ROS_WARN("%s", res.message.c_str());
        return true;  // 返回true以避免节点崩溃
    }

    try {
        // 获取传入的物体名称和支持面名称
        std::string object_name = req.object_name;
        std::string support_surface_name = req.support_surface_name;

        // 传递输入的 gripper_position 值（0~1），-1表示使用默认计算值
        pick(*arm_group, nh, object_name, support_surface_name, req.gripper_position);
        res.success = true;
        res.message = "Pick operation successful.";
    } catch (const std::exception &e) {
        res.success = false;
        res.message = e.what();
    }
    return true;
}

bool placeCallback(kinova_gen3_lite_control::Place::Request &req,
                   kinova_gen3_lite_control::Place::Response &res)
{
    ros::NodeHandle nh;

    if (!planning_scene_interface) {
        res.success = false;
        res.message = "PlanningSceneInterface is not initialized.";
        return false;
    }

    try {
        // 获取当前附着在机械臂上的物体
        auto attached_objects = planning_scene_interface->getAttachedObjects();
        if (attached_objects.empty()) {
            throw std::runtime_error("No object attached to the gripper.");
        }

        // 获取附着物体的名称
        std::string object_name = attached_objects.begin()->second.object.id;
        ROS_INFO("Attached object name is: %s", object_name.c_str());

        // 获取传入的支持面名称
        std::string support_surface_name = req.support_surface_name;

        // 确保支持面存在
        auto world_objects = planning_scene_interface->getObjects({support_surface_name});
        if (world_objects.find(support_surface_name) == world_objects.end()) {
            res.success = false;
            res.message = "Support surface " + support_surface_name + " does not exist in the planning scene.";
            ROS_WARN("%s", res.message.c_str());
            return true;
        }

        // 调用place函数执行放置
        place(*arm_group, nh, object_name, support_surface_name);
        res.success = true;
        res.message = "Place operation successful.";
    } catch (const std::exception &e) {
        res.success = false;
        res.message = e.what();
    }
    return true;
}

bool detachAndRemoveObjectCallback(kinova_gen3_lite_control::DetachAndRemoveObject::Request &req,
                                   kinova_gen3_lite_control::DetachAndRemoveObject::Response &res)
{
    try {
        // 检查物体是否附着在机械臂上
        auto attached_objects = planning_scene_interface->getAttachedObjects({req.object_id});
        if (attached_objects.find(req.object_id) != attached_objects.end()) {
            // 物体附着，执行分离操作
            arm_group->detachObject(req.object_id);
            ROS_INFO("Detached object: %s", req.object_id.c_str());
        } else {
            ROS_WARN("Object %s is not attached to the robot.", req.object_id.c_str());
        }

        // 检查物体是否存在于世界场景中
        auto world_objects = planning_scene_interface->getObjects({req.object_id});
        if (world_objects.find(req.object_id) != world_objects.end()) {
            // 物体存在，执行移除操作
            std::vector<std::string> object_ids;
            object_ids.push_back(req.object_id);
            planning_scene_interface->removeCollisionObjects(object_ids);
            ROS_INFO("Removed object: %s from world scene.", req.object_id.c_str());
        } else {
            ROS_WARN("Object %s does not exist in the planning scene.", req.object_id.c_str());
        }

        res.success = true;
        res.message = "Object detached and removed successfully.";
    } catch (const std::exception &e) {
        res.success = false;
        res.message = e.what();
    }
    return true;
}

void addCollisionObjects(std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_scene_interface, 
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
    collision_objects[0].primitive_poses[0].position.y = -0.5;
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
    collision_objects[1].primitive_poses[0].position.x = 0.5;
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
    collision_objects[2].primitive_poses[0].position.y = -0.5;
    collision_objects[2].primitive_poses[0].position.z = 0.11;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    collision_objects[2].operation = collision_objects[2].ADD;

    // 应用到规划场景
    planning_scene_interface->applyCollisionObjects(collision_objects);
}

// 回调函数：添加或删除碰撞物体
bool addRemoveCollisionObjectCallback(kinova_gen3_lite_control::AddRemoveCollisionObject::Request& req,
                                      kinova_gen3_lite_control::AddRemoveCollisionObject::Response& res)
{
    // 创建碰撞物体消息
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = req.object_id;
    collision_object.header.frame_id = "base_link";
    
    if (req.action == "add") {
        collision_object.primitives.resize(1);
        collision_object.primitives[0].type = collision_object.primitives[0].BOX;
        collision_object.primitives[0].dimensions = req.dimensions;
        collision_object.primitive_poses.resize(1);
        collision_object.primitive_poses[0] = req.pose;
        collision_object.operation = collision_object.ADD;

        // 应用碰撞物体到规划场景
        planning_scene_interface->applyCollisionObjects({collision_object});
        res.success = true;
        res.message = "Collision object added successfully.";
    } 
    else if (req.action == "remove") {
        collision_object.operation = collision_object.REMOVE;
        planning_scene_interface->applyCollisionObjects({collision_object});
        res.success = true;
        res.message = "Collision object removed successfully.";
    } 
    else {
        res.success = false;
        res.message = "Invalid action. Use 'add' or 'remove'.";
    }

    ROS_INFO("%s", res.message.c_str());
    return true;
}

// 回调函数：打印当前规划场景中的所有碰撞物体
bool printCollisionObjectsCallback(kinova_gen3_lite_control::PrintCollisionObjects::Request& req,
                                   kinova_gen3_lite_control::PrintCollisionObjects::Response& res)
{
    ros::NodeHandle nh;
    ros::ServiceClient planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                        moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    if (planning_scene_client.call(srv)) {
        const auto& objects = srv.response.scene.world.collision_objects;
        res.object_ids.resize(objects.size());
        res.poses.resize(objects.size());
        res.dimension_counts.resize(objects.size());

        // 计算总维度大小
        size_t total_dimensions = 0;
        for (const auto& obj : objects) {
            if (!obj.primitives.empty()) {
                res.dimension_counts.push_back(obj.primitives[0].dimensions.size());
                total_dimensions += obj.primitives[0].dimensions.size();
            } else {
                res.dimension_counts.push_back(0);
            }
        }

        res.dimensions.resize(total_dimensions);

        size_t idx = 0;
        for (size_t i = 0; i < objects.size(); ++i) {
            res.object_ids[i] = objects[i].id;
            res.poses[i] = objects[i].pose;

            if (!objects[i].primitives.empty()) {
                const auto& dims = objects[i].primitives[0].dimensions;
                for (double dim : dims) {
                    res.dimensions[idx++] = dim;
                }
            }
        }

        ROS_INFO("Retrieved %zu collision objects from the planning scene.", objects.size());
        return true;
    } else {
        res.object_ids.clear();
        res.poses.clear();
        res.dimensions.clear();
        res.dimension_counts.clear();
        ROS_ERROR("Failed to call service get_planning_scene");
        return false;
    }
}

bool moveArmToPoseCallback(kinova_gen3_lite_control::MoveArmToPose::Request &req,
                           kinova_gen3_lite_control::MoveArmToPose::Response &res)
{
    // 获取当前的位置
    geometry_msgs::Pose current_pose = arm_group->getCurrentPose().pose;

    // 计算目标朝向
    geometry_msgs::Quaternion target_orientation;
    if (req.orientation.x == 0.0 && req.orientation.y == 0.0 && req.orientation.z == 0.0 && req.orientation.w == 0.0) {
        // 如果 orientation 为空（即四个分量都是 0），计算默认朝向,in XOY
        geometry_msgs::Pose origin_pose, req_pose;
        origin_pose.position.x = 0.0;
        origin_pose.position.y = 0.0;
        origin_pose.position.z = 0.0;
        req_pose.position = req.position;
        tf2::Vector3 target_direction = calculateDirectionVector(origin_pose, req_pose);
        double yaw = calculateCustomYawFromDirection(target_direction);
        tf2::Quaternion tf_target_orientation = calculateOrientation(yaw);
        target_orientation = tf2::toMsg(tf_target_orientation);
    } else {
        // 使用提供的朝向，将 geometry_msgs::Quaternion 转换为 tf2::Quaternion
        tf2::Quaternion tf_req_orientation;
        tf2::fromMsg(req.orientation, tf_req_orientation);
        target_orientation = req.orientation; 
    }

    geometry_msgs::Pose target_pose = current_pose;
    target_pose.position = req.position;
    target_pose.orientation = target_orientation;
    arm_group->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        arm_group->move();
        res.success = true;
        res.message = "Arm moved to target pose successfully.";
        ROS_INFO("Arm moved to target pose successfully.");
    } else {
        res.success = false;
        res.message = "Failed to move arm to target pose.";
        ROS_INFO("Failed to move the arm to target pose.");
    }

    return true;
}

// 夹爪控制函数
bool sendGripperCommand(ros::NodeHandle n, double value) {
    ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>("/my_gen3_lite/base/send_gripper_command");
    kortex_driver::SendGripperCommand service_send_gripper_command;

    if (value < 0.0) {
        value = 0.0;
    } else if (value > 1.0) {
        value = 1.0;
    }

    kortex_driver::Finger finger;
    finger.finger_identifier = 0;
    finger.value = 1.0 - value;
    service_send_gripper_command.request.input.gripper.finger.push_back(finger);
    service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

    ROS_INFO("Setting gripper to value of close: %f", finger.value);

    if (service_client_send_gripper_command.call(service_send_gripper_command)) {
        ROS_INFO("The gripper command was sent successfully.");
    } else {
        ROS_ERROR("Failed to call service for sending gripper command");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

// 夹爪控制服务回调函数
bool gripper_control_callback(kinova_gen3_lite_control::GripperValue::Request& req, kinova_gen3_lite_control::GripperValue::Response& res) {
    ros::NodeHandle n; 
    double value = static_cast<double>(req.value);
    bool success = sendGripperCommand(n, value);

    if (success) {
        res.message = "Gripper command sent successfully.";
        res.success = true;
    } else {
        res.message = "Failed to send gripper command.";
        res.success = false;
    }
    return success;
}

// 保存姿态到文件
void savePosesToFile(const std::string& filename) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (const auto& pair : saved_poses) {
        out << YAML::Key << pair.first;
        out << YAML::Value << YAML::Flow << YAML::BeginSeq << pair.second.position.x
            << pair.second.position.y << pair.second.position.z
            << pair.second.orientation.x << pair.second.orientation.y
            << pair.second.orientation.z << pair.second.orientation.w << YAML::EndSeq;
    }
    out << YAML::EndMap;
    std::ofstream fout(filename);
    fout << out.c_str();
}

// 从文件加载姿态
void loadPosesFromFile(const std::string& filename) {
    std::ifstream fin(filename);
    if (!fin) return;
    YAML::Node node = YAML::Load(fin);
    for (YAML::const_iterator it = node.begin(); it != node.end(); ++it) {
        const std::string& name = it->first.as<std::string>();
        YAML::Node pos = it->second;
        geometry_msgs::Pose pose;
        pose.position.x = pos[0].as<double>();
        pose.position.y = pos[1].as<double>();
        pose.position.z = pos[2].as<double>();
        pose.orientation.x = pos[3].as<double>();
        pose.orientation.y = pos[4].as<double>();
        pose.orientation.z = pos[5].as<double>();
        pose.orientation.w = pos[6].as<double>();
        saved_poses[name] = pose;
    }
}

bool manage_pose_callback(kinova_gen3_lite_control::ManagePose::Request& req, kinova_gen3_lite_control::ManagePose::Response& res) {
    std::stringstream message_stream;

    switch (req.action) {
        case kinova_gen3_lite_control::ManagePose::Request::SAVE_POSE:
        {
            std::string pose_name = req.pose_name;
            if (pose_name.empty()) {
                pose_name = "pose_" + std::to_string(saved_poses.size() + 1);
            }

            if (saved_poses.find(pose_name) != saved_poses.end() && !req.overwrite) {
                res.success = false;
                res.message = "Pose name already exists. Use overwrite flag to replace it.";
            } else {
                geometry_msgs::Pose current_pose = arm_group->getCurrentPose().pose;
                saved_poses[pose_name] = current_pose;

                savePosesToFile(pose_file);  // 持久化保存到文件

                message_stream << "Saved pose with name: " << pose_name << "\n";
                message_stream << "Position - x: " << current_pose.position.x << ", y: " << current_pose.position.y << ", z: " << current_pose.position.z << "\n";
                message_stream << "Orientation - x: " << current_pose.orientation.x << ", y: " << current_pose.orientation.y << ", z: " << current_pose.orientation.z << ", w: " << current_pose.orientation.w << "\n";

                ROS_INFO_STREAM(message_stream.str()); // 输出到节点终端

                res.success = true;
                res.message = "Pose saved successfully.";
            }
            break;
        }
        case kinova_gen3_lite_control::ManagePose::Request::PRINT_POSE:
        {
            auto it = saved_poses.find(req.pose_name);
            if (it != saved_poses.end()) {
                const geometry_msgs::Pose& pose = it->second;
                message_stream << "Pose with name: " << req.pose_name << "\n";
                message_stream << "Position - x: " << pose.position.x << ", y: " << pose.position.y << ", z: " << pose.position.z << "\n";
                message_stream << "Orientation - x: " << pose.orientation.x << ", y: " << pose.orientation.y << ", z: " << pose.orientation.z << ", w: " << pose.orientation.w << "\n";

                ROS_INFO_STREAM(message_stream.str()); // 输出到节点终端

                res.success = true;
                res.message = "Pose information printed successfully.";
            } else {
                res.success = false;
                res.message = "Pose not found.";
            }
            break;
        }
        case kinova_gen3_lite_control::ManagePose::Request::PRINT_ALL_POSES:
        {
            if (saved_poses.empty()) {
                message_stream << "No poses have been saved yet.\n";
                ROS_INFO_STREAM(message_stream.str()); // 输出到节点终端
                res.success = true;
                res.message = "No poses saved.";
            } else {
                for (const auto& pair : saved_poses) {
                    const std::string& name = pair.first;
                    const geometry_msgs::Pose& pose = pair.second;
                    message_stream << "Pose with name: " << name << "\n";
                    message_stream << "Position - x: " << pose.position.x << ", y: " << pose.position.y << ", z: " << pose.position.z << "\n";
                    message_stream << "Orientation - x: " << pose.orientation.x << ", y: " << pose.orientation.y << ", z: " << pose.orientation.z << ", w: " << pose.orientation.w << "\n";
                }
                ROS_INFO_STREAM(message_stream.str()); // 输出到节点终端
                res.success = true;
                res.message = "All pose information printed successfully.";
            }
            break;
        }
        case kinova_gen3_lite_control::ManagePose::Request::MOVE_TO_POSE:
        {
            auto it = saved_poses.find(req.pose_name);
            if (it != saved_poses.end()) {
                arm_group->setPoseTarget(it->second);
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                bool success = (arm_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

                if (success) {
                    arm_group->move();
                    message_stream << "Moved to pose with name: " << req.pose_name << "\n";
                    ROS_INFO_STREAM(message_stream.str()); // 输出到节点终端
                    res.success = true;
                    res.message = "Moved to pose successfully.";
                } else {
                    res.success = false;
                    res.message = "Failed to move to pose.";
                }
            } else {
                res.success = false;
                res.message = "Pose not found.";
            }
            break;
        }
        case kinova_gen3_lite_control::ManagePose::Request::DELETE_POSE:
        {
            auto it = saved_poses.find(req.pose_name);
            if (it != saved_poses.end()) {
                saved_poses.erase(it);
                savePosesToFile(pose_file);  // 更新文件
                message_stream << "Pose with name: " << req.pose_name << " deleted successfully.\n";
                ROS_INFO_STREAM(message_stream.str()); // 输出到节点终端
                res.success = true;
                res.message = "Pose deleted successfully.";
            } else {
                res.success = false;
                res.message = "Pose not found.";
            }
            break;
        }
        case kinova_gen3_lite_control::ManagePose::Request::CLEAR_ALL_POSES:
        {
            saved_poses.clear();
            savePosesToFile(pose_file);  // 清空文件
            message_stream << "All poses cleared.\n";
            ROS_INFO_STREAM(message_stream.str()); // 输出到节点终端
            res.success = true;
            res.message = "All poses cleared.";
            break;
        }
        default:
            res.success = false;
            res.message = "Unknown action.";
            break;
    }

    // 如果有信息被打印，添加到响应的 message 中
    if (!message_stream.str().empty()) {
        res.message += "\n" + message_stream.str();
    }

    return true;
}

