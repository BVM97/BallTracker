#include <ball_tracker.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "ball_tracker");

    ballTracker bt;
    ros::spin();

    return 0;
}