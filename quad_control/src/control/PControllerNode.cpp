#include "ros/ros.h"
#include "PController.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "p_controller");
    PController ctrl;
    ctrl.run();
    return 0;
}
