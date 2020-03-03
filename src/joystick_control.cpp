#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
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

// Joystick Controller class
class Joystick_Controller {
    ros::NodeHandle nh_;
    ros::Subscriber joystick_input_;
    ros::Publisher servo_out_;
    int buttons[8];
    bool gripper_state;
    float gripper_value;
    float servo1_value;
    float servo2_value;
    float servo3_value;
    float servo1_change;
    float servo2_change;
    float servo3_change;
    

public:
    Joystick_Controller(ros::NodeHandle &nh_)
    {   
        // joystick subscriber 
        joystick_input_ = nh_.subscribe("/joy", 1, &Joystick_Controller::input_btn, this);
        // servo publisher
        servo_out_ = nh_.advertise<i2cpwm_board::ServoArray>("/servo_array", 10);

        // initial values for servos
        this->gripper_value = 420;
        this->servo1_value = 320;
        this->servo2_value = 320;
        this->servo3_value = 320;

        this->servo1_change = 2;
        this->servo2_change = 2;
        this->servo3_change = 2;
        this->gripper_state = true;
    }
    // insert the button values in an array and determine the position of the gripper
    void input_btn(const sensor_msgs::Joy& msg) {
        for(int i = 0; i < 8; i++)
        {
            this->buttons[i] = msg.buttons[i];
        }

        if(this->buttons[1] == 1)
        {
            this->gripper_state = true;
        }


        if(this->buttons[0] == 1)
        {
            this->gripper_state = false;
        }
    }

    void servo_pub()
    {
        // crete msgs needed so we can publish
        i2cpwm_board::ServoArray array;
        array.servos.clear();
        i2cpwm_board::Servo srvgrp;
        i2cpwm_board::Servo srv1;
        i2cpwm_board::Servo srv2;
        i2cpwm_board::Servo srv3;
        srvgrp.servo = 13;
        srv1.servo = 1; 
        srv2.servo = 5;
        srv3.servo = 9;
        
        // determine which servos to move according to the buttons
        if(this->buttons[2] == 1)
        {
            this->servo1_value -= servo1_change;
        }
        else if(this->buttons[3] == 1)
        {
            this->servo1_value += servo1_change;
        }
        else if(this->buttons[4] == 1)
        {
            this->servo2_value -= servo2_change;
        }
        else if(this->buttons[5] == 1)
        {
            this->servo2_value += servo2_change;
        }
        else if(this->buttons[6] == 1)
        {
            this->servo3_value -= servo3_change;
        }
        else if(this->buttons[7] == 1)
        {
            this->servo3_value += servo3_change;
        }

        if(this->gripper_state){
            this->gripper_value = GRIPPER_CLOSED;
        }else
        {
            this->gripper_value = GRIPPER_OPEN;
        }
        
        // check if servos exceed max or min values
        if(this->servo1_value > SERVO1_MAX){
            this->servo1_value = SERVO1_MAX;
        }else if(this->servo1_value < SERVO1_MIN){
            this->servo1_value = SERVO1_MIN;
        }
        if(this->servo2_value > SERVO2_MAX){
            this->servo2_value = SERVO2_MAX;
        }else if(this->servo2_value < SERVO2_MIN){
            this->servo2_value = SERVO2_MIN;
        }
        if(this->servo3_value > SERVO3_MAX){
            this->servo3_value = SERVO3_MAX;
        }else if(this->servo3_value < SERVO3_MIN){
            this->servo3_value = SERVO3_MIN;
        }

        // setup the msg
        srvgrp.value = this->gripper_value;
        srv1.value = this->servo1_value;
        srv2.value = this->servo2_value;
        srv3.value = this->servo3_value;
        array.servos.push_back(srvgrp);
        array.servos.push_back(srv2);
        array.servos.push_back(srv1);
        array.servos.push_back(srv3);
        // publish
        servo_out_.publish(array);
    }

}; // end of class definition

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "joystick_controller");
    ros::NodeHandle n;
    Joystick_Controller jc(n);
    ros::Duration timer(0.1);
    while (ros::ok())
    {
        ros::spinOnce();
        jc.servo_pub();
        timer.sleep();
  }
}