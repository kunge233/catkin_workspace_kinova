#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Empty.h"  
#include <thread>
#include <atomic>
#include "kinova_gen3_lite_control/TargetPose.h"
#include "kinova_gen3_lite_control/GripperValue.h"
#include "kinova_gen3_lite_control/ActionTrigger.h"

#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>
#include <kortex_driver/OnNotificationActionTopic.h>

#define RETRACT_ACTION_IDENTIFIER   1
#define HOME_ACTION_IDENTIFIER      2
#define PACKAGING_ACTION_IDENTIFIER 3
#define ZERO_ACTION_IDENTIFIER      4

std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};
std::atomic<bool> all_notifs_succeeded{true};

// 设置超时时间（例如10秒）
ros::Duration timeout(10);

// 全局变量来跟踪当前的action identifier 值
int current_identifier = 1000; // 从1000开始

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
  last_action_notification_id = notif.handle.identifier;
}

// 检查动作是否结束、中止或超时的函数
bool wait_for_action_end_or_abort(ros::Duration timeout)
{
  const ros::Time start_time = ros::Time::now(); // 记录开始时间

  while (ros::ok())
  {
    if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification for action %d", last_action_notification_id.load());
      return true;
    }
    else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification for action %d", last_action_notification_id.load());
      all_notifs_succeeded = false;
      return false;
    }

    // 检查是否超时
    if ((ros::Time::now() - start_time) > timeout) {
        ROS_WARN("Action timed out.");
        return false;
    }

    ros::spinOnce();
    ros::Duration(0.05).sleep(); // 稍作延时，避免CPU占用过高
  }
  return false;
}

// 通用的动作执行函数
bool execute_action_by_identifier(ros::NodeHandle n, const std::string &robot_name, int action_identifier) {
    ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
    kortex_driver::ReadAction service_read_action;

    // 设置动作标识符
    service_read_action.request.input.identifier = action_identifier;

    if (!service_client_read_action.call(service_read_action)) {
        std::string error_string = "Failed to call ReadAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    service_execute_action.request.input = service_read_action.response.output;

    if (service_client_execute_action.call(service_execute_action)) {
        ROS_INFO("The action was sent to the robot.");
        return wait_for_action_end_or_abort(timeout); // 假定这是您定义的等待动作结束或中止的函数
    } else {
        std::string error_string = "Failed to call ExecuteAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }
}

// 通用的服务回调函数
bool action_trigger_callback(kinova_gen3_lite_control::ActionTrigger::Request &req, kinova_gen3_lite_control::ActionTrigger::Response &res) {
    ros::NodeHandle n;
    std::string robot_name = "my_gen3_lite"; 

    bool success = execute_action_by_identifier(n, robot_name, req.action_identifier);

    switch (req.action_identifier) {
        case RETRACT_ACTION_IDENTIFIER:
            res.message = "RETRACT position action triggered.";
            ROS_INFO("The robot was moved to the RETRACT position.");
            break;
        case HOME_ACTION_IDENTIFIER:
            res.message = "HOME action triggered.";
            ROS_INFO("The robot was moved to the HOME position.");
            break;
        case PACKAGING_ACTION_IDENTIFIER:
            res.message = "PACKAGING action triggered.";
            ROS_INFO("The robot was moved to the PACKAGING position.");
            break;
        case ZERO_ACTION_IDENTIFIER:
            res.message = "ZERO action triggered.";
            ROS_INFO("The robot was moved to the ZERO position.");
            break;
        default:
            res.message = "Unknown action identifier.";
            success = false;
    }
    res.success = success;
    return success;
}

/**
 * 操纵机械臂到达指定姿态的函数。
 * @param n 节点句柄，用于创建服务客户端。
 * @param robot_name 机器人的名称。
 * @param constrained_pose 用户传入的受限制姿态参数。
 * @return 如果机械臂成功到达指定姿态，则返回true；否则返回false。
 */
bool cartesian_action(ros::NodeHandle n, const std::string &robot_name, const kortex_driver::ConstrainedPose &constrained_pose) { 
    // 创建服务客户端
    ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    // 清空之前可能存在的reach_pose数据
    service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();

    // 设置服务请求参数
    service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(constrained_pose);
    service_execute_action.request.input.name = "constrained_pose";
    service_execute_action.request.input.handle.identifier = current_identifier++; // pose标识符
    service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;

    // 调用服务并检查结果
    last_action_notification_event.store(0);
    if (service_client_execute_action.call(service_execute_action)) {
        ROS_INFO("Pose %d was sent to the robot.", service_execute_action.request.input.handle.identifier);
    } else {
        std::string error_string = "Failed to call ExecuteAction on pose ";
        error_string += std::to_string(service_execute_action.request.input.handle.identifier);
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    return wait_for_action_end_or_abort(timeout);
    // return true;
}

// 服务回调函数1
bool move_robot_to_pose_callback(kinova_gen3_lite_control::TargetPose::Request  &req, kinova_gen3_lite_control::TargetPose::Response &res) {
    ros::NodeHandle n;
    std::string robot_name = "my_gen3_lite"; // 机器人名称，根据实际情况修改

    // 定义机器人的姿态
    kortex_driver::ConstrainedPose constrained_pose;
    constrained_pose.target_pose.x = req.x;
    constrained_pose.target_pose.y = req.y;
    constrained_pose.target_pose.z = req.z;
    constrained_pose.target_pose.theta_x = req.theta_x;
    constrained_pose.target_pose.theta_y = req.theta_y;
    constrained_pose.target_pose.theta_z = req.theta_z;

    // 调用函数执行动作
    bool success = cartesian_action(n, robot_name, constrained_pose);

    if (success) {
        res.success = true;
        res.message = "Robot moved to the specified pose successfully.";
    } else {
        res.success = false;
        res.message = "Failed to move the robot to the specified pose.";
    }

    return true;
}

bool sendGripperCommand(ros::NodeHandle n, double value) {
    // 夹爪控制逻辑
    // Initialize the ServiceClient
    ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>("/my_gen3_lite/base/send_gripper_command");
    kortex_driver::SendGripperCommand service_send_gripper_command;

    // 限制value的值在0到1之间
    if (value < 0.0) {
        value = 0.0;
    } else if (value > 1.0) {
        value = 1.0;
    }

    // Initialize the request
    kortex_driver::Finger finger;
    finger.finger_identifier = 0; // 根据需要设置手指标识符
    finger.value = value; // 设置夹爪开合程度
    service_send_gripper_command.request.input.gripper.finger.push_back(finger);
    service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

    // 打印夹爪将要到达的闭合程度
    ROS_INFO("Setting gripper to value of close: %f", finger.value);

    if (service_client_send_gripper_command.call(service_send_gripper_command)) {
        ROS_INFO("The gripper command was sent successfully.");

    } else {
        std::string error_string = "Failed to call service for sending gripper command";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

bool gripper_control_callback(kinova_gen3_lite_control::GripperValue::Request& req, kinova_gen3_lite_control::GripperValue::Response& res) {
    ros::NodeHandle n; 
    // 调用sendGripperCommand函数，传入从服务请求中获取的值
    double value = static_cast<double>(req.value);
    bool success = sendGripperCommand(n, value);

    if (success) {
        res.message = "Gripper command sent successfully.";
        res.success = true;
        return true;
    } else {
        res.message = "Failed to send gripper command.";
        res.success = false;
        return false;
    }
}

void publishStopCommand() {
    // 停止指令
    std::string stop_topic = "/my_gen3_lite/in/stop";

    static ros::Publisher stop_pub;
    if (!stop_pub) {
        stop_pub = ros::NodeHandle().advertise<std_msgs::Empty>(stop_topic, 10);
    }
        
    std_msgs::Empty stop_command;
    stop_pub.publish(stop_command);
    ROS_INFO("Stop command published!");
}

bool stop_robot_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    publishStopCommand();
    ROS_INFO("Robot stop service called.");
    res.message = "Stop robot successfully.";
    res.success = true;
    return true; 
}

void clearFaultsCommand() {
    // 停止指令
    std::string clear_faults_topic = "/my_gen3_lite/in/clear_faults";

    static ros::Publisher clear_faults_pub;
    if (!clear_faults_pub) {
        clear_faults_pub = ros::NodeHandle().advertise<std_msgs::Empty>(clear_faults_topic, 10);
    }
        
    std_msgs::Empty clear_faults_command;
    clear_faults_pub.publish(clear_faults_command);
    ROS_INFO("Clear faults command published!");
}

bool clear_faults_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    clearFaultsCommand();
    ROS_INFO("Clear Robot faults service called.");
    res.message = "Clear faults successfully.";
    res.success = true;
    return true; 
}

// This function sets the reference frame to the robot's base
bool set_cartesian_reference_frame(ros::NodeHandle n, const std::string &robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Set reference to base frame.");
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;

    std::string robot_name = "my_gen3_lite";

    kortex_driver::ConstrainedPose my_constrained_pose;
    // 定义速度限制
    kortex_driver::CartesianSpeed my_cartesian_speed;
    my_cartesian_speed.translation = 0.1f;
    my_cartesian_speed.orientation = 15.0f;
    my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

    // //test position
    // my_constrained_pose.target_pose.x = 0.374f;
    // my_constrained_pose.target_pose.y = 0.081f;
    // my_constrained_pose.target_pose.z = 0.450f;
    // my_constrained_pose.target_pose.theta_x = -57.6f;
    // my_constrained_pose.target_pose.theta_y = 91.1f;
    // my_constrained_pose.target_pose.theta_z = 2.3f;

    // 激活动作通知
    ros::ServiceClient service_client_activate_notif = n.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
    kortex_driver::OnNotificationActionTopic service_activate_notif;
    if (service_client_activate_notif.call(service_activate_notif))
    {
        ROS_INFO("Action notification activated!");
    }
    else 
    {
        std::string error_string = "Action notification publication failed";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }
    ros::Duration(1.00).sleep();

    ros::Subscriber sub = n.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);
    ROS_INFO("Subscribed to action_topic");

    // 设置笛卡尔参考系
    set_cartesian_reference_frame(n, robot_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // 创建move_robot_to_pose服务器
    ros::ServiceServer move_service = n.advertiseService("move_robot_to_pose", move_robot_to_pose_callback);
    ROS_INFO("Ready to move robot to specified pose.");

    // 注册Home服务+其他服务，使用相应的动作标识符
    ros::ServiceServer action_service = n.advertiseService("action_trigger", action_trigger_callback);

    // 夹爪控制服务
    ros::ServiceServer gripper_service = n.advertiseService("gripper_control", gripper_control_callback);

    // stop服务器
    ros::ServiceServer stop_service = n.advertiseService("stop_robot", stop_robot_callback);
    
    // clear faults服务器
    ros::ServiceServer clear_faults_service = n.advertiseService("clear_faults", clear_faults_callback);

    // 等待服务请求
    ros::spin();

    return 0;
}