
#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <Encoder.h>

/* Node for reading quadrature encoders and publishing a joinstate message through rosserial
 *
 */


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

Encoder leftEnc(2, 6);
Encoder rightEnc(3, 7);


ros::NodeHandle nh;

sensor_msgs::JointState rightFMsg;
sensor_msgs::JointState rightRMsg;
sensor_msgs::JointState leftFMsg;
sensor_msgs::JointState leftRMsg;
ros::Publisher pubFR("/py_controller/front_right_wheel/encoder", &rightFMsg);
ros::Publisher pubRR("/py_controller/rear_right_wheel/encoder", &rightRMsg);
ros::Publisher pubFL("/py_controller/front_left_wheel/encoder", &leftFMsg);
ros::Publisher pubRL("/py_controller/rear_left_wheel/encoder", &leftRMsg);



void setup() {
  nh.initNode();
  nh.advertise(pubFR);
  nh.advertise(pubFL);
  nh.advertise(pubFL);
  nh.advertise(pubRL);

  char *name0[] = {"front_right_wheel"};
  char *name1[] = {"rear_right_wheel"};
  char *name2[] = {"front_left_wheel"};
  char *name3[] = {"rear_left_wheel"};
  rightFMsg.name = name0;
  rightRMsg.name = name1;
  leftFMsg.name = name2;
  leftRMsg.name = name3;
}

long oldRPosition  = -999;
long oldLPosition  = -999;

void loop() {

  //read encoders
  long newLPosition = leftEnc.read();
  long newRPosition = rightEnc.read();
  if ((newLPosition != oldLPosition) || (newRPosition != oldRPosition)) {
    oldLPosition = newLPosition;
    oldRPosition = newRPosition;

    float posR[]={(float)newRPosition};
    
    rightFMsg.position = posR;

    pubFR.publish(&rightFMsg);

    nh.spinOnce();
  }
  
}
