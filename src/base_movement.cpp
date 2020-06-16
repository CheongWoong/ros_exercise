#include <iostream>

#include <geometry_msgs/Twist.h>

#include "../include/unist_tb3_kb/control_node.hpp"
#include "../include/unist_tb3_kb/base_movement.hpp"



BaseMovement::BaseMovement(ControlNode& control_node_) :
  control_node(control_node_),
  target_linear_vel(0.0),
  target_angular_vel(0.0),
  control_linear_vel(0.0),
  control_angular_vel(0.0)
{
  // assume the base stops at the beginning
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.linear.z = 0.0;
  twist_msg.angular.x = 0.0;
  twist_msg.angular.y = 0.0;
  twist_msg.angular.z = 0.0;
}


BaseMovement::~BaseMovement() {
  // do nothing
}


void BaseMovement::step() {

  makeSimpleProfile(control_linear_vel, target_linear_vel, LIN_VEL_STEP_SIZE/2.0);
  twist_msg.linear.x = control_linear_vel;

  makeSimpleProfile(control_angular_vel, target_angular_vel, ANG_VEL_STEP_SIZE/2.0);
  twist_msg.angular.z = control_angular_vel;

  control_node.setBaseMovement(twist_msg);

}


void BaseMovement::moveForward(double step_size) {
  target_linear_vel += step_size;
  if (target_linear_vel > WAFFLE_MAX_LIN_VEL) target_linear_vel = WAFFLE_MAX_LIN_VEL;
  step();
}


void BaseMovement::moveBackward(double step_size) {
  target_linear_vel -= step_size;
  if (target_linear_vel < -WAFFLE_MAX_LIN_VEL) target_linear_vel = -WAFFLE_MAX_LIN_VEL;
  step();
}


void BaseMovement::turnLeft(double step_size) {
  target_angular_vel += step_size;
  if (target_angular_vel > WAFFLE_MAX_ANG_VEL) target_angular_vel = WAFFLE_MAX_ANG_VEL;
  step();
}


void BaseMovement::turnRight(double step_size) {
  target_angular_vel -= step_size;
  if (target_angular_vel < -WAFFLE_MAX_ANG_VEL) target_angular_vel = -WAFFLE_MAX_ANG_VEL;
  step();
}


void BaseMovement::stopNow() {

  control_node.stopBaseMovement();

  target_linear_vel = 0.0;
  target_angular_vel = 0.0;
  control_linear_vel = 0.0;
  control_angular_vel = 0.0;
}


void BaseMovement::setLinearVelocity(double vel) {
  if (vel > WAFFLE_MAX_LIN_VEL) vel = WAFFLE_MAX_LIN_VEL;
  if (vel < -WAFFLE_MAX_LIN_VEL) vel = -WAFFLE_MAX_LIN_VEL;
  target_linear_vel = vel;
  control_linear_vel = vel;
  step();
}


void BaseMovement::setAngularVelocity(double vel) {
  if (vel > WAFFLE_MAX_ANG_VEL) vel = WAFFLE_MAX_ANG_VEL;
  if (vel < -WAFFLE_MAX_ANG_VEL) vel = -WAFFLE_MAX_ANG_VEL;
  target_angular_vel = vel;
  control_angular_vel = vel;
  step();
}


void BaseMovement::makeSimpleProfile(double& control, double target, double step) {
  if (target > control) {
     control += step;
     if (control > target) control = target;
  } else if (target < control) {
     control -= step;
     if (control < target) control = target;
  } // target == control => do nothing
}


