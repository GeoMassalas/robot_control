#include <ros/ros.h>
#include <cmath>
#include <i2cpwm_board/ServoArray.h>
#include <i2cpwm_board/Servo.h>
#include <std_srvs/Empty.h>
#include <image_processing/Point2D.h>
#include <image_processing/PointTP.h>


#define PI 3.14159265

/*      Range of servos
    Servo 1 is the base 
    Servo 2 is the 1st link
    Servo 3 is the 2nd link
    Gripper is open @ 420 and closed @ 100
*/

using namespace std;

static const float SERVO1_MAX = 420;
static const float SERVO1_MIN = 80;

static const float SERVO2_MAX = 410;
static const float SERVO2_MIN = 220;

static const float SERVO3_MAX = 410;
static const float SERVO3_MIN = 220;

static const float GRIPPER_OPEN = 420;
static const float GRIPPER_CLOSED = 260;

static const float LINK1 = 160;
static const float LINK2 = 147;
static const float TOOL = 100;

static const float FRAME_X = 129;
static const float FRAME_Y = 152;

static const float PLATFORM_RIGHT = 93;
static const float PLATFORM_LEFT = 384;

static const float SERVO2_RANGE_MIN = 410;
static const float SERVO2_RANGE_MAX = 290;

static const float SERVO3_RANGE_MIN = 380;
static const float SERVO3_RANGE_MAX = 275;

static const float SERVO1_INIT = 420;
static const float SERVO2_INIT = 305;
static const float SERVO3_INIT = 225;

static const float SERVO1_DROP = 420;
static const float SERVO2_DROP = 305;
static const float SERVO3_DROP = 225;

static const float TICK_TIMER = 0.02;

class Robot_Controller {
    ros::NodeHandle nh_;
    ros::Publisher servo_out_;
    ros::ServiceClient pos_client;
    ros::ServiceClient ss_client;
    float servo1_value;
    float servo2_value;
    float servo3_value;
    float servo_gripper_value;
    float servo1_change;
    float servo2_change;
    float servo3_change;
    float servo_gripper_change;
    

public:
    Robot_Controller(ros::NodeHandle &nh_)
    {   
        // stop servos client
        ss_client = nh_.serviceClient<std_srvs::Empty>("/stop_servos");
        pos_client = nh_.serviceClient<image_processing::PointTP>("/image_processing/publish");

        // servo publisher
        servo_out_ = nh_.advertise<i2cpwm_board::ServoArray>("/servos_absolute", 10);

        // initial values for servos
        this->servo1_value = SERVO1_INIT;
        this->servo2_value = SERVO2_INIT;
        this->servo3_value = SERVO3_INIT;
        this->servo1_change = 2;
        this->servo2_change = 2;
        this->servo3_change = 2;
        this->servo_gripper_change = 4;
    }

    // insert the button values in an array and determine the position of the gripper
    void position_check() {
        image_processing::PointTP srv;
        if (pos_client.call(srv))
        {
            if(srv.response.obj_detected)
            {
                ros::Duration tick_timer(TICK_TIMER);
                ros::Duration timer_2(0.4);
                double x_tr,y_tr, r, theta;
                ROS_INFO("Object x: %.0f  y: %.0f out of %d objects recieved!", (double)srv.response.point.x, (double)srv.response.point.y, srv.response.obj_num);
                x_tr = (double)srv.response.point.x - FRAME_X;
                y_tr = (double)srv.response.point.y + FRAME_Y;
                r = sqrt(x_tr*x_tr + y_tr*y_tr);
                theta = asin(abs(x_tr)/r)  * 180.0 / PI;
                double a,b,c,theta_a,theta_c;
                c = LINK1;
                a = LINK2;
                b = r-TOOL;
                theta_c = acos((a*a + b*b - c*c)/(2*a*b)) * 180 / PI; // servo 3
                theta_a = acos((c*c + b*b - a*a)/(2*c*b)) * 180 / PI; // servo 2
                ROS_INFO("Calculated angles:\tservo1: %.2f \tservo2: %.2f \tservo3: %.2f ", theta, theta_a, theta_c);
                // map angles to servo values
                if( x_tr < 0) {
                    theta *= -1;
                }
                float f = map(theta, -40, 40, PLATFORM_RIGHT, PLATFORM_LEFT);
                float f_a = map(theta_a, 45, 90, SERVO2_RANGE_MIN, SERVO2_RANGE_MAX);
                float f_c = map(theta_c, 0, 65, SERVO3_RANGE_MIN, SERVO3_RANGE_MAX);

                toggle_gripper(0);
                ROS_INFO("MOVING SERVO_1 to %.1f , theta: %.1f", f, theta);
                move_to_position(f, this->servo2_value, this->servo3_value);
                timer_2.sleep();                
                ROS_INFO("Opening Grip.");
                toggle_gripper(1);
                timer_2.sleep();
                ROS_INFO("MOVING SERVO_2 to %.1f , theta: %.1f", f_a, theta_a);
                ROS_INFO("MOVING SERVO_3 to %.1f , theta: %.1f", f_c, theta_c);
                move_to_position(this->servo1_value, f_a, f_c);
                timer_2.sleep();
                ROS_INFO("Closing Grip.");
                toggle_gripper(0);
                timer_2.sleep();
                ROS_INFO("Moving to Initial position");
                move_to_position(SERVO1_INIT, SERVO2_INIT, SERVO3_INIT);
                ROS_INFO("Initial position found!");
                ROS_INFO("Opening Grip.");
                toggle_gripper(1);
                timer_2.sleep();
                stop_servos();
                ROS_INFO("Stoping Servos");
                ROS_INFO("Loop Complete!");
            }
            else
                ROS_INFO("No objects detected!");
        }
        else
        {
            ROS_ERROR("Failed to call service publish");
        }
    }

    void move_to_position(float pos1, float pos2, float pos3) {
        int steps = 0;
        while(true) {
            ros::Duration tick(TICK_TIMER);
            if(abs(this->servo1_value - pos1) <= servo1_change){
                this->servo1_value = pos1;
            } else if (this->servo1_value - pos1 > 0) {
                this->servo1_value -= servo1_change;
            } else if (this->servo1_value - pos1 < 0) {
                this->servo1_value += servo1_change;
            }

            if(abs(this->servo2_value - pos2) <= servo2_change){
                this->servo2_value = pos2;
            } else if (this->servo2_value - pos2 > 0) {
                this->servo2_value -= servo2_change;
            } else if (this->servo2_value - pos2 < 0) {
                this->servo2_value += servo2_change;
            }

            if(abs(this->servo3_value - pos3) <= servo3_change){
                this->servo3_value = pos3;
            } else if (this->servo3_value - pos3 > 0) {
                this->servo3_value -= servo3_change;
            } else if (this->servo3_value - pos3 < 0) {
                this->servo3_value += servo3_change;
            }
            // debug
            // ROS_INFO("Servo values: servo1: %.2f servo2: %.2f servo3: %.2f ", this->servo1_value, this->servo2_value, this->servo3_value);

            servo_pub_(1, this->servo1_value, SERVO1_MIN, SERVO1_MAX);
            servo_pub_(5, this->servo2_value, SERVO2_MIN, SERVO2_MAX);
            servo_pub_(9, this->servo3_value, SERVO3_MIN, SERVO3_MAX);
            tick.sleep();
            steps++;
            if(steps > 300){ break; } 
        }
    }

    void toggle_gripper(int i) {
        if(i == 0) {
            while(this->servo_gripper_value > GRIPPER_CLOSED) {
                this->servo_gripper_value -= this->servo_gripper_change;
                servo_pub_(13, this->servo_gripper_value, GRIPPER_CLOSED, GRIPPER_OPEN);
            }
            ROS_INFO("Gripper Closed");
        } else {
            servo_pub_(13, GRIPPER_OPEN, GRIPPER_CLOSED, GRIPPER_OPEN);
            this->servo_gripper_value = GRIPPER_OPEN;
            ROS_INFO("Gripper Open");
        }
    }

    void stop_servos() {
        std_srvs::Empty empty;
        ss_client.call(empty);
    }

    void servo_pub_(int servo, float value, float servo_min, float servo_max)
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
        // ROS_INFO("Servo No: %d  Value: %f", srv.servo, srv.value);
    }

    float map(double &x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

}; // end of class definition



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle n;
    Robot_Controller rc(n);
    ros::Duration timer(5.0);
    while (ros::ok())
    {
        ros::spinOnce();
        rc.position_check();
        timer.sleep();
    }
}
