#include <iostream>
#include <iomanip>
#include <string>

#include "../include/unist_tb3_kb/control_node.hpp"
#include "../include/unist_tb3_kb/posix_terminal.hpp"
#include "../include/unist_tb3_kb/base_movement.hpp"
#include "../include/unist_tb3_kb/arm_movement.hpp"
#include "../include/unist_tb3_kb/gripper_movement.hpp"
#include "../include/unist_tb3_kb/yolo_control.hpp"


using namespace::std;


void print_main_menu() {
  cout << "Please select a command:" << endl;

  cout << "  w     - base moves forward" << endl;
  cout << "  x     - base moves backward" << endl;
  cout << "  a     - base turns left" << endl;
  cout << "  d     - base turns right" << endl;
  cout << "  s     - base stops now" << endl;

/*
  cout << "  left  - arm moves forward" << endl;
  cout << "  right - arm moves backward" << endl;
  cout << "  up    - arm moves up" << endl;
  cout << "  down  - arm moves down" << endl;
  cout << "  [     - arm turns left" << endl;
  cout << "  ]     - arm turns right" << endl;
*/

  cout << "  enter - arm's all-zero pose" << endl;
//  cout << "  i     - arm's zero-z pose" << endl;
  cout << "  g     - arm's low gripping pose" << endl;
  cout << "  h     - arm's home pose" << endl;
  cout << "  p     - arm's state" << endl;

  cout << "  space - gripper open/close" << endl;
  cout << "  1-9   - gripper opening" << endl;

  cout << "  y     - yolo automatically pickup the object" << endl;
  cout << "  l     - yolo look at the object" << endl;
  cout << "  m     - yolo move toward the looked object" << endl;
  cout << "  \\     - yolo's status" << endl;

  cout << "  ?     - show this menu again" << endl;
  cout << "  q     - quit" << endl;
  cout << endl;
}


int main(int argc, char **argv) {

  terminal_init();

  ControlNode control_node(argc, argv);

  if (!control_node.init()) {
    ROS_ERROR("Cannot initialize the connection to TurtleBot.");
    return -1;
  }

  BaseMovement base_movement(control_node);
  ArmMovement arm_movement(control_node, false);
  GripperMovement gripper_movement(control_node);
  YoloControl yolo_control(control_node, base_movement, arm_movement, gripper_movement);


  cout << fixed << setprecision(6);
  set_terminal_raw_mode();


  print_main_menu();

  bool isContinue = true;

  while(isContinue) {

    while(true) {
      if (wait_for_key_pressed(100)) break;
      base_movement.step();
      arm_movement.step();
    }

    char c = getKeyStroke();

    switch(c) {

      case 'w':
        base_movement.moveForward();
        break;
      case 'x':
        base_movement.moveBackward();
        break;
      case 'a':
        base_movement.turnLeft();
        break;
      case 'd':
        base_movement.turnRight();
        break;
      case 's':
        base_movement.stopNow();
        break;

/*
      case LEFT_ARROW_KEY:
        arm_movement.moveForward();
        break;
      case RIGHT_ARROW_KEY:
        arm_movement.moveBackward();
        break;
      case UP_ARROW_KEY:
        arm_movement.moveUp();
        break;
      case DOWN_ARROW_KEY:
        arm_movement.moveDown();
        break;
      case '[':
        arm_movement.turnLeft();
        break;
      case ']':
        arm_movement.turnRight();
        break;
*/

      case '\n':
        arm_movement.setAllZeroPose();
        break;
/*
      case 'i':
        arm_movement.setZeroZPose();
        break;
*/
      case 'g':
        arm_movement.setLowGrippingPose();
        break;
      case 'h':
        arm_movement.setHomePose();
        break;
      case 'p':
        arm_movement.printState();
        break;

      case ' ':
        gripper_movement.toggleGripperOpening();
        break;
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9': 
        gripper_movement.setGripperAngleLevel(c - '1');
        break;

      case 'y':
        yolo_control.autoPickupTheObject();
        break;
      case 'l':
        yolo_control.lookAtTheObject();
        break;
      case 'm':
        yolo_control.moveTowardTheLookedObject();
        break;
      case '\\':
        yolo_control.showYoloStatus();
        break;

      case '?':
        print_main_menu();
        break;
      case 'q':
        isContinue = false;
        break;
      default:
        cout << "Error: unknown command \'" << c << "\'" << endl;
    }
  }

  set_terminal_original_mode();

  cout << "Bye." << endl;

  return 0;
}

