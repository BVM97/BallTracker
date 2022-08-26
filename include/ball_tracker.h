// Ball tracking algrotihm by Adrian Rosebock implement in C++ and ROS 1.
// For the original article, see: https://pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/sync_policies/exact_time.h>
// #include <message_filters/time_synchronizer.h>

// NOTE: distance b/w left and right cam ZED2 deduced from /tf_static topic (see tranform of child frame w.r.t. frame)

typedef boost::shared_ptr<cv_bridge::CvImage const> CvImageConstPtr;

class ballTracker{

private:

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber gray_image_sub;
    image_transport::Publisher image_pub;

    std::array<int, 3> colorLow, colorHigh; // HSV

public:

    ballTracker(): it(nh) {
        gray_image_sub = it.subscribe("/zed2/zed_node/left/image_rect_color", 1, &ballTracker::imgCallback, this);
        image_pub = it.advertise("/ball_tracker/tracked_ball", 1);

        // colorLow = {161, 155, 51};
        // colorHigh = {146, 247, 237};

        colorLow = {113, 155, 51};
        colorHigh = {103, 247, 237};
    };

    void imgCallback(const sensor_msgs::ImageConstPtr& msg){
        
        cv_bridge::CvImagePtr rgb_ptr;
        try{
            rgb_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

        cv::Mat trackMask = trackBall(rgb_ptr->image);

        sensor_msgs::ImagePtr out_msg;
        out_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", trackMask).toImageMsg();
        image_pub.publish(out_msg);


    }

    cv::Mat trackBall(const cv::Mat &img){

        // Pre-processing:
        cv::Mat smallImg, blurImg, hsvImg;
        cv::resize(img, smallImg, cv::Size(img.cols*0.5, img.rows*0.5));
        cv::GaussianBlur(smallImg, blurImg, cv::Size(11, 11), 0);
        cv::cvtColor(blurImg, hsvImg, cv::COLOR_BGR2HSV);

        // Color masking:
        cv::Mat mask;
        cv::inRange(hsvImg, cv::Scalar(colorLow[0], colorLow[1], colorLow[2]), cv::Scalar(colorHigh[0], colorHigh[1], colorHigh[2]), mask);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        return mask;

    };

};