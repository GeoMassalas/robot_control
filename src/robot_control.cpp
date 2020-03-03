#include <ros/ros.h>
#include <i2cpwm_board/ServoArray.h>
#include <i2cpwm_board/Servo.h>
/*      Range of servos
    Servo 1 is the base 
    Servo 2 is the 1st link
    Servo 3 is the 2nd link
    Gripper is open @ 420 and closed @ 100
*/
static const float SERVO1_MAX = 420;
static const float SERVO1_MIN = 120;

static const float SERVO2_MAX = 420;
static const float SERVO2_MIN = 120;

static const float SERVO3_MAX = 420;
static const float SERVO3_MIN = 120;

static const float GRIPPER_OPEN = 420;
static const float GRIPPER_CLOSED = 100;



int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "joystick_controller");
    ros::NodeHandle n;
    ros::Duration timer(0.1);
    while (ros::ok())
    {
        ros::spinOnce();
        jc.servo_pub();
        timer.sleep();
  }
}