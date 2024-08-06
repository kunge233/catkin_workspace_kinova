#include <cmath>
#include <memory>
#include <algorithm>
#include <ros/ros.h>
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

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// 全局变量声明
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group;
std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

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
          ros::NodeHandle& node_handle)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    geometry_msgs::Pose object_pose, origin_pose;
    std::vector<double> object_dimensions;

    origin_pose.position.x = 0.0;
    origin_pose.position.y = 0.0;
    origin_pose.position.z = 0.0;

    // 使用GetPlanningScene服务获取最新场景信息
    getObjectInfo(node_handle, "object", object_pose, object_dimensions);

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

    // 设置抓取时的夹爪姿态
    double gripper_position = std::max(object_dimensions[0] * 5 - 0.06, 0.0);
    closedGripper(grasps[0].grasp_posture, gripper_position);

    // 设置支持面为 table1
    move_group.setSupportSurfaceName("table1");

    // 调用 pick 函数执行抓取
    ROS_INFO("Picking............");
    move_group.pick("object", grasps);
    ROS_INFO("Pick Finished.^-^");
}

void place(moveit::planning_interface::MoveGroupInterface& move_group,
           ros::NodeHandle& node_handle)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    geometry_msgs::Pose object_pose, origin_pose;
    std::vector<double> object_dimensions;

    origin_pose.position.x = 0.0;
    origin_pose.position.y = 0.0;
    origin_pose.position.z = 0.0;

    // 使用GetPlanningScene服务获取最新场景信息
    getObjectInfo(node_handle, "table2", object_pose, object_dimensions);

    // 获取当前机械臂的姿态
    geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
    tf2::Quaternion current_orientation;
    tf2::fromMsg(current_pose.orientation, current_orientation);
    // 打印姿态的四元数
    // ROS_INFO("current Quaternion: [x: %f, y: %f, z: %f, w: %f]",
    //         current_orientation.x(), current_orientation.y(),
    //         current_orientation.z(), current_orientation.w());

    // 计算接近方向
    tf2::Vector3 approach_direction = calculateDirectionVector(origin_pose, object_pose);
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
    place_location[0].place_pose.pose.position = object_pose.position;
    place_location[0].place_pose.pose.position.z += object_dimensions[2] / 2 + 0.08;

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

    // 设置支撑表面为table2
    move_group.setSupportSurfaceName("table2");
    // 调用 place 函数将对象放置到指定的位置
    ROS_INFO("Placing............");
    move_group.place("object", place_location);
    ROS_INFO("Place Finished.^-^");
}

bool pickCallback(kinova_gen3_lite_control::Pick::Request &req,
                  kinova_gen3_lite_control::Pick::Response &res)
{
    ros::NodeHandle nh;
    try {
        pick(*arm_group, nh);
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
    try {
        place(*arm_group, nh);
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
        // Detach the object from the robot
        arm_group->detachObject(req.object_id);
        ROS_INFO("Detached object: %s", req.object_id.c_str());

        // Remove the object from the planning scene
        std::vector<std::string> object_ids;
        object_ids.push_back(req.object_id);
        planning_scene_interface->removeCollisionObjects(object_ids);
        ROS_INFO("Removed object: %s", req.object_id.c_str());

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