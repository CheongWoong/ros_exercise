#include <iostream>

#include "../include/unist_tb3_kb/posix_terminal.hpp"
#include "../include/unist_tb3_kb/control_node.hpp"
#include "../include/unist_tb3_kb/yolo_control.hpp"


// Settings for XPS13

#define LOCK_TIMEOUT        19.0
#define LOOK_AT_TIMEOUT     80.0
#define MOVE_TOWARD_TIMEOUT 60.0

#define IMAGE_SIZE_X     1280
#define IMAGE_SIZE_Y     960
#define CENTER_THRESHOLD 15

#define TARGET_BOX_WIDTH    240
#define BOX_WIDTH_THRESHOLD 15

#define YOLO_BASE_LINEAR_VEL   0.015
#define YOLO_BASE_ANGULAR_VEL  0.008


/*

// Settings for MacBook Pro

#define LOCK_TIMEOUT        6.0
#define LOOK_AT_TIMEOUT     50.0
#define MOVE_TOWARD_TIMEOUT 50.0

#define IMAGE_SIZE_X     1280
#define IMAGE_SIZE_Y     960
#define CENTER_THRESHOLD 50

#define TARGET_BOX_WIDTH    240
#define BOX_WIDTH_THRESHOLD 15

#define YOLO_BASE_LINEAR_VEL   0.016
#define YOLO_BASE_ANGULAR_VEL  0.026

*/


YoloControl::YoloControl(ControlNode& control_node_, BaseMovement& base_movement_, ArmMovement& arm_movement_, GripperMovement& gripper_movement_) :
  control_node(control_node_),
  base_movement(base_movement_),
  arm_movement(arm_movement_),
  gripper_movement(gripper_movement_),
  last_image_stamp(0.001)
{
  // do nothing
}


YoloControl::~YoloControl() {
  // do nothing
}


void YoloControl::autoPickupTheObject() {

  cout << "Start picking up the object automatically" << endl;

  if (!arm_movement.setHomePose()) {
    cout << "FAILED: cannot set arm's home pose" << endl;
    return;
  }

  ros::Duration(0.2).sleep();

  if (!lookAtTheObject()) {
    cout << "FAILED: cannot look at the object" << endl;
    return;
  }

  ros::Duration(0.2).sleep();

  if (!moveTowardTheLookedObject()) {
    cout << "FAILED: cannot move toward the looked object" << endl;
    return;
  }

  ros::Duration(0.2).sleep();

  if (!gripper_movement.openGripper()) {
    cout << "FAILED: cannot open the gripper" << endl;
    return;
  }

  ros::Duration(0.2).sleep();

  if (!arm_movement.setLowGrippingPose()) {
    cout << "FAILED: cannot set arm's low gripping pose" << endl;
    return;
  }

  ros::Duration(7.0).sleep();

  if (!gripper_movement.closeGripper()) {
    cout << "FAILED: cannot close the gripper" << endl;
    return;
  }

  ros::Duration(0.4).sleep();

  if (!arm_movement.setAllZeroPose()) {
    cout << "FAILED: cannot set arm's all-zero pose" << endl;
    return;
  }

}


bool YoloControl::lookAtTheObject() {
  ros::Time time_begin = ros::Time::now() ;

  base_movement.stopNow();
  int moveDirection = 0;  //  Left:+1  Right:-1  Stop:0

  cout << "Start looking at the object" << endl;

  while ((ros::Time::now() - time_begin).toSec() <= LOOK_AT_TIMEOUT) {

    if (wait_for_key_pressed(0)) {
      if (getKeyStroke() == ESC_KEY) {
        cout << "ABORTED" << endl;
        return false;
      }
    }

    lockTheObject();
    if (isTheObjectOutdated()) {
      base_movement.stopNow();
      cout << "FAIL: cannot lock the object" << endl;
      return false;
    }

    int mid = (last_bounding_box.xmin + last_bounding_box.xmax) / 2;

    if (mid < (IMAGE_SIZE_X / 2) - CENTER_THRESHOLD) {  // need to turn left
      cout << "Turn left: center error = " << mid - (IMAGE_SIZE_X / 2) << endl;
      double v = YOLO_BASE_ANGULAR_VEL;
      if (moveDirection == 0) {
        base_movement.setAngularVelocity(v);
        moveDirection = 1;
      } else if (moveDirection < 0) {
        base_movement.stopNow();
        base_movement.setAngularVelocity(v);
        moveDirection = 1;
      } else {
        base_movement.setAngularVelocity(v);
      }
    } else if (mid > (IMAGE_SIZE_X / 2) + CENTER_THRESHOLD) { // need to turn right
      cout << "Turn right: center error = " << mid - (IMAGE_SIZE_X / 2) << endl;
      double v = -YOLO_BASE_ANGULAR_VEL;
      if (moveDirection == 0) {
        base_movement.setAngularVelocity(v);
        moveDirection = -1;
      } else if (moveDirection > 0) {
        base_movement.stopNow();
        base_movement.setAngularVelocity(v);
        moveDirection = -1;
      } else {
        base_movement.setAngularVelocity(v);
      }
    } else {
      base_movement.stopNow();
      cout << "SUCCESS: center error = " << mid - (IMAGE_SIZE_X / 2) << endl;
      return true;
    }

    ros::Duration(0.2).sleep();
  }

  base_movement.stopNow();
  cout << "FAIL: cannot look at the object with time limit" << endl;
  return false;
}


bool YoloControl::moveTowardTheLookedObject() {
  ros::Time time_begin = ros::Time::now() ;

  base_movement.stopNow();

  while ((ros::Time::now() - time_begin).toSec() <= MOVE_TOWARD_TIMEOUT) {

    if (wait_for_key_pressed(0)) {
      if (getKeyStroke() == ESC_KEY) {
        cout << "ABORTED" << endl;
        return false;
      }
    }

    lockTheObject();
    if (isTheObjectOutdated()) {
      base_movement.stopNow();
      cout << "FAIL: cannot lock the object" << endl;
      return false;
    }

    int w = last_bounding_box.xmax - last_bounding_box.xmin;

    if (w < TARGET_BOX_WIDTH - BOX_WIDTH_THRESHOLD) {
      cout << "Move forward: box width error = " << w - TARGET_BOX_WIDTH << endl;
      base_movement.setLinearVelocity(YOLO_BASE_LINEAR_VEL);
    } else if (w > TARGET_BOX_WIDTH + BOX_WIDTH_THRESHOLD) {
      cout << "Move backward: box width error = " << w - TARGET_BOX_WIDTH << endl;
      base_movement.setLinearVelocity(-YOLO_BASE_LINEAR_VEL/4);
    } else {
      base_movement.stopNow();
      cout << "SUCCESS: box width error = " << w - TARGET_BOX_WIDTH << endl;
      return true;
    } 

    ros::Duration(0.2).sleep();
  }

  base_movement.stopNow();
  cout << "FAIL: cannot move toward the object with time limit" << endl;
  return false;
}



void YoloControl::showYoloStatus() {
  lockTheObject();
  if (isTheObjectOutdated()) {
    cout << "YOLO STATUS: cannot lock the object" << endl;
  } else {
    int mid = (last_bounding_box.xmin + last_bounding_box.xmax) / 2;
    cout << "YOLO STATUS: center error = " << mid - (IMAGE_SIZE_X / 2) << "  ";
    cout << "box width error = " << (last_bounding_box.xmax - last_bounding_box.xmin) - TARGET_BOX_WIDTH << endl;
  }
}


bool YoloControl::lockTheObject() {
  darknet_ros_msgs::BoundingBoxes msg = control_node.getYoloLastMsg();

  if (msg.image_header.stamp <= last_image_stamp) return false; // wrong time stamp

  if (msg.bounding_boxes.size() == 0) return false;  // cannot see any object

  long largest_size = 0;
  darknet_ros_msgs::BoundingBox box_with_largest_size;

  for (auto b = msg.bounding_boxes.begin(); b != msg.bounding_boxes.end(); ++b) {
    if (b->Class == "bottle" || b->Class == "vase" || b->Class == "cup" || b->Class == "fire hydrant") {
      long size = (b->xmax - b->xmin) * (b->ymax - b->ymin);
      if (size > largest_size) {
        largest_size = size;
        box_with_largest_size = *b;
      }
    }
  }

  if (largest_size > 0) {
    last_image_stamp = msg.image_header.stamp;
    last_bounding_box = box_with_largest_size;
    return true;
  } else {
    return false;
  }
}


bool YoloControl::isTheObjectOutdated() {
  if ((ros::Time::now() - last_image_stamp).toSec() > LOCK_TIMEOUT) {
    cout << "Image delay = " << (ros::Time::now() - last_image_stamp).toSec() << " s" << endl;
    return true;
  } else {
    return false;
  }
}

