// glove_controller_class.cpp
#include "ros/ros.h"
#include "std_msgs/String.h" 
#include "std_msgs/Empty.h"  
#include <boost/bind.hpp>
#include <kortex_driver/TwistCommand.h>
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

//linear speed m/s
#define LSPEED_1 0.05
#define LSPEED_2 0.05
#define LSPEED_3 0.05
//angular speed rad/s
#define ASPEED_1 0.1
#define ASPEED_2 0.2
#define ASPEED_3 0.5

std::string robot_name = "my_gen3_lite";

class GloveController {
public:
    GloveController() {
        // 初始化发布者和订阅者
        usbDataSub = nh_.subscribe("usb_data", 10, &GloveController::gloveCommandCallback, this);
        pub = nh_.advertise<kortex_driver::TwistCommand>("/my_gen3_lite/in/cartesian_velocity", 10);
        stop_pub = nh_.advertise<std_msgs::Empty>("/my_gen3_lite/in/stop", 10);
    }

    ~GloveController() {
        // 清理资源
    }

    void run() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub;
    ros::Publisher stop_pub;
    ros::Subscriber usbDataSub;
    kortex_driver::TwistCommand twistCommand;

    void gloveCommandCallback(const std_msgs::String::ConstPtr& msg) {
        std::string instruction = msg->data;
        ROS_INFO("Received command: [%s]", instruction.c_str());

        if (instruction.length() != 6) {
            ROS_ERROR("Invalid command format. Command should be 6 characters long.");
            publishStopCommand();
            return;
        }

        char type = instruction[0];
        char axis = instruction[1];
        std::string value_str = instruction.substr(2);
        int value = std::stoi(value_str);
        char speedLevel_str = instruction[2];
        int speedLevel = speedLevel_str - '0';  // 获取挡位数字
        double fingerValue = 1.0;

        kortex_driver::TwistCommand twistCommand;
        twistCommand.reference_frame = 0;
        twistCommand.duration = 0; //command duration time

        switch (type) {
            case 'M':  // 平动指令
            if (axis == 'X' || axis == 'Y' || axis == 'Z') {
                double speed = 0.0;  // 定义速度变量

                // 挡位0表示停止
                if (speedLevel == 0) {
                    // 无需设置速度，保持speed为0.0
                } else if (speedLevel <= 3) {
                    speed = getSpeedByLevel(speedLevel);
                } else if (speedLevel >= 4 && speedLevel <= 6) {
                    // 反方向速度，速度与正方向1-3档相同，但是添加负号
                    speed = -getSpeedByLevel(speedLevel - 3); 
                }

                // 根据轴设置线速度
                twistCommand.twist.linear_x = (axis == 'X') ? speed : 0.0;
                twistCommand.twist.linear_y = (axis == 'Y') ? speed : 0.0;
                twistCommand.twist.linear_z = (axis == 'Z') ? speed : 0.0;

                // 重置角速度
                twistCommand.twist.angular_x = 0.0;
                twistCommand.twist.angular_y = 0.0;
                twistCommand.twist.angular_z = 0.0;
            }
            break;
            case 'R':  // 转动指令
                if (axis == 'X' || axis == 'Y' || axis == 'Z') {
                    // 计算角度差值，并确定角速度的正负
                    double angularSpeed = (value > 10 || value < -10) ? (value > 0 ? ASPEED_2 : -ASPEED_2) : 0.0;
                    // 根据轴逐个设置角速度，并确保其他轴的角速度为0
                    twistCommand.twist.angular_x = (axis == 'X') ? angularSpeed : 0.0;
                    twistCommand.twist.angular_y = (axis == 'Y') ? angularSpeed : 0.0;
                    twistCommand.twist.angular_z = (axis == 'Z') ? angularSpeed : 0.0;
                    
                    // 重置线速度
                    twistCommand.twist.linear_x = 0.0;
                    twistCommand.twist.linear_y = 0.0;
                    twistCommand.twist.linear_z = 0.0;
                }
            break;
            case 'G': {
                // 夹爪指令
                // 计算变换后的value
                fingerValue = 1 - 0.01 * value;
                // 如果结果为负数，转为0
                if (fingerValue < 0) {
                    fingerValue = 0;
                }
                if (fingerValue > 1) {
                    fingerValue = 1;
                }
                sendGripperCommand(fingerValue);
            } 
                break;
            default:
                ROS_ERROR("Invalid command type.");
                publishStopCommand();
                return;
        }

        // 在发布之前打印即将发布的指令详情
        ROS_INFO("Publishing command:");
        ROS_INFO("Linear Velocity: x: %f, y: %f, z: %f", 
                twistCommand.twist.linear_x, 
                twistCommand.twist.linear_y, 
                twistCommand.twist.linear_z);
        ROS_INFO("Angular Velocity: x: %f, y: %f, z: %f", 
                twistCommand.twist.angular_x, 
                twistCommand.twist.angular_y, 
                twistCommand.twist.angular_z);
        ROS_INFO("Gripper Position: %f", fingerValue);        
        // 确保已经创建了发布者
        static ros::Publisher pub;
        if (!pub) {
            pub = ros::NodeHandle().advertise<kortex_driver::TwistCommand>("/my_gen3_lite/in/cartesian_velocity", 10);
        }
        // 发布指令
        pub.publish(twistCommand);
    }

    double getSpeedByLevel(int level) {
        // 挡位速度
        switch (level) {
        case 1: return 0.0;
        case 2: return LSPEED_2;
        case 3: return LSPEED_3;
        default: return 0.0;  // 如果挡位无效，返回0
        }
    }

    void publishStopCommand() {
        // 停止指令
        std::string stop_topic = "/my_gen3_lite/in/stop";
        
        // 确保已经创建了发布者
        static ros::Publisher stop_pub;
        if (!stop_pub) {
            stop_pub = ros::NodeHandle().advertise<std_msgs::Empty>(stop_topic, 10);
        }
        
        std_msgs::Empty stop_command;
        stop_pub.publish(stop_command);
        ROS_INFO("Stop command published!");
    }

    bool sendGripperCommand(double value) {
        // 夹爪控制逻辑
        // Initialize the ServiceClient
        ros::ServiceClient service_client_send_gripper_command = nh_.serviceClient<kortex_driver::SendGripperCommand>("/my_gen3_lite/base/send_gripper_command");
        kortex_driver::SendGripperCommand service_send_gripper_command;

        // Initialize the request
        kortex_driver::Finger finger;
        finger.finger_identifier = 0; // 根据需要设置手指标识符
        finger.value = value; // 设置夹爪开合程度
        service_send_gripper_command.request.input.gripper.finger.push_back(finger);
        service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

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

    // 其他私有函数...
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_controller");

    GloveController controller;
    controller.run();
    return 0;
}