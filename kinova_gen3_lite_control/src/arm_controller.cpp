#include "ros/ros.h"
#include "std_msgs/String.h"
#include <thread>
#include <atomic>

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

#define HOME_ACTION_IDENTIFIER 2

bool all_notifs_succeeded = true;

std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};

// 全局变量来跟踪当前的 identifier 值
int current_identifier = 1000; // 从1000开始

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
  last_action_notification_id = notif.handle.identifier;
}

bool wait_for_action_end_or_abort()
{
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
    ros::spinOnce();
    ros::Duration(0.1).sleep(); // 稍作延时，避免CPU占用过高
  }
  return false;
}

// 让机械臂回到home位置的函数
bool home_the_robot(ros::NodeHandle n, const std::string &robot_name)
{
    // 读取Home动作
    ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
    kortex_driver::ReadAction service_read_action;

    // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

    if (!service_client_read_action.call(service_read_action))
    {
        std::string error_string = "Failed to call ReadAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    // 执行读取到的Home动作
    ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    service_execute_action.request.input = service_read_action.response.output;
    
    if (service_client_execute_action.call(service_execute_action))
    {
        ROS_INFO("The Home position action was sent to the robot.");
    }
    else
    {
        std::string error_string = "Failed to call ExecuteAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    return wait_for_action_end_or_abort();
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
    last_action_notification_event = 0;
    if (service_client_execute_action.call(service_execute_action)) {
        ROS_INFO("Pose %d was sent to the robot.", service_execute_action.request.input.handle.identifier);
    } else {
        std::string error_string = "Failed to call ExecuteAction on pose ";
        error_string += std::to_string(service_execute_action.request.input.handle.identifier);
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    wait_for_action_end_or_abort();

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
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;

    std::string robot_name = "my_gen3_lite";
    bool success = true;

    kortex_driver::ConstrainedPose my_constrained_pose;
    // 定义速度限制
    kortex_driver::CartesianSpeed my_cartesian_speed;
    my_cartesian_speed.translation = 0.1f;
    my_cartesian_speed.orientation = 15.0f;
    my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

    //test position
    my_constrained_pose.target_pose.x = 0.374f;
    my_constrained_pose.target_pose.y = 0.081f;
    my_constrained_pose.target_pose.z = 0.450f;
    my_constrained_pose.target_pose.theta_x = -57.6f;
    my_constrained_pose.target_pose.theta_y = 91.1f;
    my_constrained_pose.target_pose.theta_z = 2.3f;

    // We need to call this service to activate the Action Notification on the kortex_driver node.
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

      // Run the example
    success &= set_cartesian_reference_frame(n, robot_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    success &= home_the_robot(n, robot_name);
    success &= cartesian_action(n, robot_name, my_constrained_pose);
    success &= all_notifs_succeeded;
    
    return success ? 0 : 1;

}