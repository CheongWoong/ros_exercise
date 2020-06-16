#include <vector>
#include <string>
#include <sstream>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <ros/network.h>
#include <std_msgs/String.h>

#include "../include/unist_tb3_kb/control_node.hpp"


ControlNode::ControlNode(int argc, char** argv) 
    :init_argc(argc), init_argv(argv)
{
  // do nothing
}


ControlNode::~ControlNode() {
  if (ros::isStarted()) {
    stopBaseMovement();  // stop the movement
    delete nh;
    delete spinner;
    ros::shutdown();
    ros::waitForShutdown();
  }
}


bool ControlNode::init() {

  ros::init(init_argc, init_argv, "unist_tb3_kb");

  if ( !ros::master::check() ) {
    return false;
  }

  nh = new ros::NodeHandle();

  cmd_vel_pub = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);

  yolo_bounding_boxes_sub = nh->subscribe("/darknet_ros/bounding_boxes", 100, &ControlNode::yoloBoundingBoxesCallback, this);

  spinner = new ros::AsyncSpinner(4);
  spinner->start();

  move_group_arm = new moveit::planning_interface::MoveGroupInterface("arm");
  move_group_gripper = new moveit::planning_interface::MoveGroupInterface("gripper");

  return true;
}


void ControlNode::setBaseMovement(const geometry_msgs::Twist& twist_msg) {
  cmd_vel_pub.publish(twist_msg);
}


void ControlNode::stopBaseMovement() {
  geometry_msgs::Twist twist_msg;

  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;

  cmd_vel_pub.publish(twist_msg);
}


vector<double> ControlNode::getArmJointAngles() {
  return move_group_arm->getCurrentJointValues();
}


bool ControlNode::setArmJointSpacePath(vector<double> joint_angle, double path_time) {

  const robot_state::JointModelGroup* joint_model_group =
    move_group_arm->getCurrentState()->getJointModelGroup("arm");

  moveit::core::RobotStatePtr current_state = move_group_arm->getCurrentState();

  vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = joint_angle.at(0);  // in radian
  joint_group_positions[1] = joint_angle.at(1);  // in radian
  joint_group_positions[2] = joint_angle.at(2);  // in radian
  joint_group_positions[3] = joint_angle.at(3);  // in radian
  move_group_arm->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_arm->move();

  return true;
}


vector<double> ControlNode::getArmPosition() {
  geometry_msgs::Pose current_pose = move_group_arm->getCurrentPose().pose;
  vector<double> temp_position;
  temp_position.push_back(current_pose.position.x);
  temp_position.push_back(current_pose.position.y);
  temp_position.push_back(current_pose.position.z);
  return temp_position;
}


bool ControlNode::setArmTaskSpacePath(vector<double> position, double path_time, bool isKeepGripperLevel) {
  if (isKeepGripperLevel) {
    moveit_msgs::OrientationConstraint oc;
    oc.link_name = "end_effector_link";
    oc.header.frame_id = "link1";
    oc.orientation.w = 1.0;
    oc.absolute_x_axis_tolerance = 0.1;
    oc.absolute_y_axis_tolerance = 0.1;
    oc.absolute_z_axis_tolerance = 3.14;
    oc.weight = 1.0;
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(oc);
    move_group_arm->setPathConstraints(constraints);
  }

  move_group_arm->setPositionTarget(position[0], position[1], position[2]);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_arm->move();

  return true;
}


double ControlNode::getGripperAngle() {
  vector<double> jointValues = move_group_gripper->getCurrentJointValues();
  return jointValues[0];
}


bool ControlNode::setGripperAngle(double angle) {

  const robot_state::JointModelGroup* joint_model_group =
    move_group_gripper->getCurrentState()->getJointModelGroup("gripper");

  moveit::core::RobotStatePtr current_state = move_group_gripper->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  joint_group_positions[0] = angle;  // in radian
  move_group_gripper->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_gripper->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_gripper->move();

  return true;
}


void ControlNode::yoloBoundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes& msg) {
  lock_guard<mutex> lck(yolo_mtx);
  last_yolo_msg = msg;
}


const darknet_ros_msgs::BoundingBoxes ControlNode::getYoloLastMsg() {
  lock_guard<mutex> lck(yolo_mtx);
  return last_yolo_msg;
}


