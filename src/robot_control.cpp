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
static const float SERVO1_MIN = 100;

static const float SERVO2_MAX = 410;
static const float SERVO2_MIN = 220;

static const float SERVO3_MAX = 410;
static const float SERVO3_MIN = 220;

static const float GRIPPER_OPEN = 420;
static const float GRIPPER_CLOSED = 100;

static const float LINK1 = 135;
static const float LINK2 = 147;
static const float TOOL = 100;

static const float FRAME_X = 130;
static const float FRAME_Y = 150;

class Robot_Controller {
    ros::NodeHandle nh_;
    ros::Publisher servo_out_;
    ros::ServiceClient pos_client;
    ros::ServiceClient ss_client;
    float gripper_value;
    float servo1_value;
    float servo2_value;
    float servo3_value;
    

public:
    Robot_Controller(ros::NodeHandle &nh_)
    {   
        // stop servos client
        ss_client = nh_.serviceClient<std_srvs::Empty>("/stop_servos");
        pos_client = nh_.serviceClient<image_processing::PointTP>("/image_processing/publish");

        // servo publisher
        servo_out_ = nh_.advertise<i2cpwm_board::ServoArray>("/servos_absolute", 10);

        // initial values for servos
        this->servo1_value = 320;
        this->servo2_value = 320;
        this->servo3_value = 320;
    }

    // insert the button values in an array and determine the position of the gripper
    void position_check() {
        image_processing::PointTP srv;
        if (pos_client.call(srv))
        {
            if(srv.response.obj_detected)
            {
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
                ROS_INFO("Calculated angles\n\tservo1: %.2f \n\tservo2: %.2f \n\tservo3: %.2f ", theta, theta_a, theta_c);
            }
            else
                ROS_INFO("No objects detected!");
        }
        else
        {
            ROS_ERROR("Failed to call service publish");
        }
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