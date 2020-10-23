#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <i2cpwm_board/ServoArray.h>
#include <i2cpwm_board/Servo.h>
#include <std_srvs/Empty.h>
#include <algorithm>

/*      Range of servos
    Servo 1 is the base 
    Servo 2 is the 1st link
    Servo 3 is the 2nd link
    Gripper is open @ 420 and closed @ 100
*/

static const float SERVO1_MAX = 420;
static const float SERVO1_MIN = 100;

static const float SERVO2_MAX = 410;
static const float SERVO2_MIN = 220;

static const float SERVO3_MAX = 410;
static const float SERVO3_MIN = 220;

static const float GRIPPER_MAX = 420;
static const float GRIPPER_MIN = 250;

static const float SERVO1_INIT = 420;
static const float SERVO2_INIT = 225;
static const float SERVO3_INIT = 305;

// Joystick Controller class
class Joystick_Controller {
    ros::NodeHandle nh_;
    ros::Subscriber joystick_input_;
    ros::Publisher servo_out_;
    ros::ServiceClient client;
    int buttons[9];
    float servo1_value;
    float servo2_value;
    float servo3_value;
    float servo_gripper_value;
    float servo1_change;
    float servo2_change;
    float servo3_change;
    float servo_gripper_change;
    

public:
    Joystick_Controller(ros::NodeHandle &nh_)
    {   
        // stop servos client
        client = nh_.serviceClient<std_srvs::Empty>("/stop_servos");
        // joystick subscriber 
        joystick_input_ = nh_.subscribe("/joy", 1, &Joystick_Controller::input_btn, this);
        // servo publisher
        servo_out_ = nh_.advertise<i2cpwm_board::ServoArray>("/servos_absolute", 10);

        // initial values for servos
        this->servo1_value = SERVO1_INIT;
        this->servo2_value = SERVO2_INIT;
        this->servo3_value = SERVO3_INIT;
        this->servo_gripper_value = 260;
        this->servo1_change = 1;
        this->servo2_change = 1;
        this->servo3_change = 1;
        this->servo_gripper_change = 2;
        std::fill_n(this->buttons, 9, -1);
        std_srvs::Empty empty;
        client.call(empty);
    }
    // insert the button values in an array and determine the position of the gripper
    void input_btn(const sensor_msgs::Joy& msg) {
        for(int i = 0; i < 9; i++)
        {
            this->buttons[i] = msg.buttons[i];
        }
    }

    void servo_pub()
    {   
        // determine which servos to move according to the buttons
        if(this->buttons[2] == 1)
        {
            this->servo1_value -= servo1_change;
            servo_pub_(1, servo1_value, SERVO1_MIN, SERVO1_MAX);
        }
        else if(this->buttons[3] == 1)
        {
            this->servo1_value += servo1_change;
            servo_pub_(1, servo1_value, SERVO1_MIN, SERVO1_MAX);
        }
        else if(this->buttons[4] == 1)
        {
            this->servo2_value -= servo2_change;
            servo_pub_(5, servo2_value, SERVO2_MIN, SERVO2_MAX);
        }
        else if(this->buttons[5] == 1)
        {
            this->servo2_value += servo2_change;
            servo_pub_(5, servo2_value, SERVO2_MIN, SERVO2_MAX);
        }
        else if(this->buttons[6] == 1)
        {
            this->servo3_value -= servo3_change;
            servo_pub_(9, servo3_value, SERVO3_MIN, SERVO3_MAX);
        }
        else if(this->buttons[7] == 1)
        {
            this->servo3_value += servo3_change;
            servo_pub_(9, servo3_value, SERVO3_MIN, SERVO3_MAX);
        }else if(this->buttons[1] == 1){
            this->servo_gripper_value += servo_gripper_change;
            servo_pub_(13, servo_gripper_value, GRIPPER_MIN, GRIPPER_MAX);
        }else if(this->buttons[0] == 1)
        {
            this->servo_gripper_value -= servo_gripper_change;
            servo_pub_(13, servo_gripper_value, GRIPPER_MIN, GRIPPER_MAX);
        }else if(this->buttons[8] == 1)
        {
            std_srvs::Empty empty;
            client.call(empty);
        }
    }

    void servo_pub_(int servo, float &value, float servo_min, float servo_max)
    {
        i2cpwm_board::ServoArray array;
        i2cpwm_board::Servo srv;
        srv.servo = servo;
        if(value > servo_max)
            value = servo_max;
        else if(value < servo_min){
            value = servo_min;
        }
        srv.value = value;
        array.servos.push_back(srv);
        servo_out_.publish(array);
        // Debug msg
        ROS_INFO("Servo No: %d  Value: %f", srv.servo, srv.value);
    }

}; // end of class definition

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "joystick_controller");
    ros::NodeHandle n;
    Joystick_Controller jc(n);
    ros::Duration timer(0.01);
    while (ros::ok())
    {
        ros::spinOnce();
        jc.servo_pub();
        timer.sleep();
  }
}