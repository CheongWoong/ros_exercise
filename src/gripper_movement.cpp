#include <iostream>

#include "../include/unist_tb3_kb/control_node.hpp"
#include "../include/unist_tb3_kb/gripper_movement.hpp"


GripperMovement::GripperMovement(ControlNode& control_node_) :
  control_node(control_node_)
{
  // do nothing
}


GripperMovement::~GripperMovement() {
  // do nothing
}


void GripperMovement::toggleGripperOpening() {
  if (control_node.getGripperAngle() < (MAX_GRIPPER_ANGLE + MIN_GRIPPER_ANGLE) / 2.0) {
    control_node.setGripperAngle(MAX_GRIPPER_ANGLE);
  } else {
    control_node.setGripperAngle(MIN_GRIPPER_ANGLE);
  }
}


bool GripperMovement::setGripperAngleLevel(int level) {
  double angle = level * (MAX_GRIPPER_ANGLE - MIN_GRIPPER_ANGLE) / GRIPPER_ANGLE_LEVEL + MIN_GRIPPER_ANGLE;
  if (angle > MAX_GRIPPER_ANGLE) angle = MAX_GRIPPER_ANGLE;
  if (angle < MIN_GRIPPER_ANGLE) angle = MIN_GRIPPER_ANGLE;
  return control_node.setGripperAngle(angle);
}

