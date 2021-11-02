#include "ros/ros.h"
#include "franka_controllers/whole_body_controller.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"wholebody_test");
    ros::NodeHandle nh;
    
    franka_controllers::TestClass myHello;
    myHello.run();

    return 0;
}