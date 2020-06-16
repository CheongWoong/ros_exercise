#include <iostream>
#include <cmath>

#include "../include/unist_tb3_kb/control_node.hpp"
#include "../include/unist_tb3_kb/arm_movement.hpp"

#define JOINT_EPS 0.2

#define PATH_TIME 2.0

#define MOVE_X_STEP 0.05
#define MOVE_Z_STEP 0.025

#define TURN_STEP 0.3875
#define MAX_TURN_ANGLE 3.1



ArmMovement::ArmMovement(ControlNode& control_node_, bool isKeepGripperLevel_) :
  control_node(control_node_),
  isKeepGripperLevel(isKeepGripperLevel_)
{
  target_joint_angle = control_node.getArmJointAngles();
  target_position = control_node.getArmPosition();
  last_pose = -1;
}


ArmMovement::~ArmMovement() {
  // do nothing
}


void ArmMovement::step() {
  target_joint_angle = control_node.getArmJointAngles();
  target_position = control_node.getArmPosition();
}


bool ArmMovement::setAllZeroPose() {
  vector<double> joint_angle{0.0, 0.0, 0.0, 0.0};

  target_joint_angle = control_node.getArmJointAngles();

  if (last_pose != 0 && isJointDiffEnough(joint_angle, target_joint_angle)) {
    target_joint_angle = joint_angle;
    if (control_node.setArmJointSpacePath(target_joint_angle, PATH_TIME)) {
      target_position = control_node.getArmPosition();
      last_pose = 0;
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}


bool ArmMovement::setZeroZPose() {
  target_joint_angle[0] = 0.0;
  bool result = control_node.setArmJointSpacePath(target_joint_angle, PATH_TIME);
  target_position = control_node.getArmPosition();
  return result;
}


bool ArmMovement::setLowGrippingPose() {
  vector<double> joint_angle{0.0, 1.021631, -0.168738, -0.811476};
  
  target_joint_angle = control_node.getArmJointAngles();

  if (last_pose != 1 && isJointDiffEnough(joint_angle, target_joint_angle)) {
    target_joint_angle = joint_angle;
    if (control_node.setArmJointSpacePath(target_joint_angle, PATH_TIME)) {
      target_position = control_node.getArmPosition();
      last_pose = 1;
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}


bool ArmMovement::setHomePose() {
  vector<double> joint_angle{0.0, -1.0, 0.3, 0.7};
  
  target_joint_angle = control_node.getArmJointAngles();

  if (last_pose != 2 && isJointDiffEnough(joint_angle, target_joint_angle)) {
    target_joint_angle = joint_angle;
    if (control_node.setArmJointSpacePath(target_joint_angle, PATH_TIME)) {
      target_position = control_node.getArmPosition();
      last_pose = 2;
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}


bool ArmMovement::moveForward() { 
  target_position[0] += MOVE_X_STEP;
  bool result = control_node.setArmTaskSpacePath(target_position, PATH_TIME, isKeepGripperLevel);
  target_joint_angle = control_node.getArmJointAngles();
  return result;
}


bool ArmMovement::moveBackward() { 
  target_position[0] -= MOVE_X_STEP;
  bool result = control_node.setArmTaskSpacePath(target_position, PATH_TIME, isKeepGripperLevel);
  target_joint_angle = control_node.getArmJointAngles();
  return result;
}


bool ArmMovement::moveUp() { 
  target_position[2] += MOVE_Z_STEP;
  bool result = control_node.setArmTaskSpacePath(target_position, PATH_TIME, isKeepGripperLevel);
  target_joint_angle = control_node.getArmJointAngles();
  return result;
}


bool ArmMovement::moveDown() { 
  target_position[2] -= MOVE_Z_STEP;
  bool result = control_node.setArmTaskSpacePath(target_position, PATH_TIME, isKeepGripperLevel);
  target_joint_angle = control_node.getArmJointAngles();
  return result;
}


bool ArmMovement::turnLeft() {
  target_joint_angle[0] += TURN_STEP;
  if (target_joint_angle[0] > MAX_TURN_ANGLE) target_joint_angle[0] = MAX_TURN_ANGLE;
  bool result = control_node.setArmJointSpacePath(target_joint_angle, PATH_TIME);
  target_position = control_node.getArmPosition();
  return result;
}


bool ArmMovement::turnRight() {
  target_joint_angle[0] -= TURN_STEP;
  if (target_joint_angle[0] < -MAX_TURN_ANGLE) target_joint_angle[0] = -MAX_TURN_ANGLE;
  bool result = control_node.setArmJointSpacePath(target_joint_angle, PATH_TIME);
  target_position = control_node.getArmPosition();
  return result;
}


void ArmMovement::printState() {

  cout << "Arm's target joint angles:  ";
  for (auto i = target_joint_angle.begin(); i != target_joint_angle.end(); ++i) {
    cout << *i << ' ';
  }
  cout << endl;

  vector<double> arm_joint_angle = control_node.getArmJointAngles();
  cout << "Arm's current joint angles: ";
  for (auto i = arm_joint_angle.begin(); i != arm_joint_angle.end(); ++i) {
    cout << *i << ' ';
  }
  cout << endl;

  cout << "Arm's target position:  ";
  for (auto i = target_position.begin(); i != target_position.end(); ++i) {
    cout << *i << ' ';
  }
  cout << endl;

  vector<double> arm_position = control_node.getArmPosition();
  cout << "Arm's current position: ";
  for (auto i = arm_position.begin(); i != arm_position.end(); ++i) {
    cout << *i << ' ';
  }
  cout << endl;

  cout << endl;
}


bool ArmMovement::isJointDiffEnough(vector<double>& joint_angle_1, vector<double>& joint_angle_2) {
  for (int i=0; i<4; i++) {
    if (abs(joint_angle_1[i] - joint_angle_2[i]) >= JOINT_EPS) {
      return true;
    }
  }
  return false;
}


