#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "lvio_ros_msgs/CorrectData.h"
#include "lvio_ros_msgs/PointCloud3.h"

using namespace message_filters;

void vision_handler(const lvio_ros_msgs::PointCloud3ConstPtr& img, const lvio_ros_msgs::CorrectDataConstPtr& pose, const sensor_msgs::PointCloudConstPtr& cloud)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "synchronizer");
    ros::NodeHandle nh;
    
    message_filters::Subscriber<lvio_ros_msgs::PointCloud3> sub_imgs(nh, "/vins_estimator/feature", 100);
    message_filters::Subscriber<lvio_ros_msgs::CorrectData> sub_correct_data(nh, "/vins_estimator/correct_data", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_vision_local_cloud(nh, "/vins_estimator/vision_local_cloud", 100);

    TimeSynchronizer<lvio_ros_msgs::PointCloud3, 
                     lvio_ros_msgs::CorrectData, 
                     sensor_msgs::PointCloud> sync_vision(sub_imgs, sub_correct_data, sub_vision_local_cloud, 1000);
    sync_vision.registerCallback(boost::bind(&vision_handler, _1, _2, _3));

    ros::spin();
    return 0;
}