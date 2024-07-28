#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from kinova_gen3_lite_control.srv import TargetPose, TargetPoseResponse
from kinova_gen3_lite_control.srv import NamedPose, NamedPoseResponse
from kinova_gen3_lite_control.srv import GetPose, GetPoseResponse
from kinova_gen3_lite_control.srv import GripperValue, GripperValueResponse

class ArmControllerMoveIt(object):
    """ArmControllerMoveIt"""
    
    def __init__(self):
        super(ArmControllerMoveIt, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_controller_moveit_node', anonymous=True)

        try:
            # 获取参数
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
                raise ValueError("Gripper joint names are not provided.")
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 6)

            # 初始化 MoveIt! 接口对象
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.arm_group.set_planner_id("RRTConnectkConfigDefault")  # 设置规划器
            self.arm_group.set_planning_time(10)  # 设置规划超时时间（秒）
            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20
            )

            rospy.loginfo("MoveIt! interface initialized.")

            # 如果有夹爪，初始化夹爪组
            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
                rospy.loginfo("Gripper interface initialized.")

            # 创建服务
            self.move_service = rospy.Service('move_robot_to_pose', TargetPose, self.handle_move_robot_to_pose)
            rospy.loginfo("Service 'move_robot_to_pose' ready to receive requests.")

            self.named_pose_service = rospy.Service('move_to_named_pose', NamedPose, self.handle_move_to_named_pose)

            self.get_pose_service = rospy.Service('get_cartesian_pose', GetPose, self.handle_get_cartesian_pose)

            self.set_gripper_position_service = rospy.Service('set_gripper_position', GripperValue, self.handle_set_gripper_position)
            
            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except rospy.ROSException as e:
            rospy.logerr("ROS Exception during initialization: %s", str(e))
            self.is_init_success = False
        except moveit_commander.MoveItCommanderException as e:
            rospy.logerr("MoveIt! Commander Exception during initialization: %s", str(e))
            self.is_init_success = False
        except ValueError as e:
            rospy.logerr("Value Error: %s", str(e))
            self.is_init_success = False
        except Exception as e:
            rospy.logerr("Unexpected Exception during initialization: %s", str(e))
            self.is_init_success = False
        else:
            self.is_init_success = True
            rospy.loginfo("Initialization successful.")

    def handle_move_robot_to_pose(self, req):
        target_pose = Pose()
        target_pose.position.x = req.x
        target_pose.position.y = req.y
        target_pose.position.z = req.z

        # For quaternion orientation (w, x, y, z), you may need to convert from Euler angles if needed
        # Here, assuming you directly use the provided x, y, z as orientation
        target_pose.orientation.x = req.theta_x
        target_pose.orientation.y = req.theta_y
        target_pose.orientation.z = req.theta_z

        self.arm_group.set_pose_target(target_pose)

        # Plan and execute
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        response = TargetPoseResponse()
        response.success = success
        if success:
            rospy.loginfo("Movement to the target pose was successful.")
            response.message = "Movement to the target pose was successful."
        else:
            rospy.logwarn("Failed to move to the target pose.")
            response.message = "Failed to move to the target pose."

        return response
    
    def handle_move_to_named_pose(self, req):
        response = NamedPoseResponse()
        try:
            success = self.reach_named_position(req.target)
            response.success = success
            if success:
                rospy.loginfo("Moved to the named pose: " + req.target)
                response.message = "Moved to the named pose: " + req.target
            else:
                rospy.logwarn("Failed to move to the named pose: " + req.target)
                response.message = "Failed to move to the named pose: " + req.target
        except Exception as e:
            rospy.logerr("Error moving to the named pose: %s", str(e))
            response.success = False
            response.message = str(e)

        return response

    def reach_named_position(self, target):
        #target: home, retract, vertical
        arm_group = self.arm_group
        
        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        return arm_group.execute(trajectory_message, wait=True)
    
    def handle_get_cartesian_pose(self, req):
        response = GetPoseResponse()
        try:
            pose = self.get_cartesian_pose()
            response.pose = pose
            rospy.loginfo("Returned current cartesian pose.")
        except Exception as e:
            rospy.logerr("Error getting cartesian pose: %s", str(e))
        return response
    
    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose
    
    def handle_set_gripper_position(self, req):
        response = GripperValueResponse()
        try:
            success = self.reach_gripper_position(req.value)
            response.success = success
            if success:
                rospy.loginfo("Gripper moved to the position: " + str(req.value))
                response.message = "Gripper moved to the position: " + str(req.value)
            else:
                rospy.logwarn("Failed to move the gripper to the position: " + str(req.value))
                response.message = "Failed to move the gripper to the position: " + str(req.value)
        except Exception as e:
            rospy.logerr("Error moving gripper to the position: %s", str(e))
            response.success = False
            response.message = str(e)

        return response
    
    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 
            
def main():
    try:
        controller = ArmControllerMoveIt()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
