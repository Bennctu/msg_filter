#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <algorithm>
// #include <list>
#include <vector>
#include "lvio_ros_msgs/CorrectData.h"
#include "lvio_ros_msgs/PointCloud3.h"
#include "lvio_ros_msgs/Td.h"
using namespace message_filters;
using namespace std;

void vision_handler(const lvio_ros_msgs::CorrectDataConstPtr &pose, const sensor_msgs::PointCloudConstPtr &img, const sensor_msgs::PointCloudConstPtr &cloud, const lvio_ros_msgs::TdConstPtr &td)
{
    // std::cout << "Number of 2D feature:   " << img->channels[0].values.size() << std::endl;
    // std::cout << "Number of cloud point is:" << cloud->channels.size() << std::endl;
    // ROS_INFO("img: %f", img->header.stamp.toSec());
    // ROS_INFO("pose: %f", pose->header.stamp.toSec());
    int match_cnt = 0;
    for (unsigned int i = 0; i < img->channels[0].values.size(); i++)
    {
        int v = img->channels[0].values[i] + 0.5;
        int feature_id = v / 1;
        int cnt = 0;
        /*for (auto it = cloud->channels[0].values.begin(); it != cloud->channels[0].values.end(); it++)
        {
            if (*it == feature_id)
            {
                match_cnt++;
                std::cout << "feature_id:" << feature_id << std::endl;
                cnt++;
                //break;
            }
        }*/
        auto it = find(cloud->channels[0].values.begin(), cloud->channels[0].values.end(), feature_id);
        if (it == cloud->channels[0].values.end())
            ROS_WARN("Fail to find %f", *it);
        else
        {
            int index = it - cloud->channels[0].values.begin();
            cout << "point: " << cloud->points[index] << endl;
        }
        
        if (cnt > 1) ROS_WARN("It doesn't make sense!");
    }


    std::cout << "match count" << match_cnt << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "synchronizer");
    ros::NodeHandle nh;

    // message_filters::Subscriber<lvio_ros_msgs::PointCloud3> sub_imgs(nh, "/vins_estimator/feature", 100);
    message_filters::Subscriber<lvio_ros_msgs::CorrectData> sub_correct_data(nh, "/vins_estimator/correct_data", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_vision_local_cloud(nh, "/vins_estimator/vision_local_cloud", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud> sub_imgs(nh, "/feature_tracker/feature", 100);
    message_filters::Subscriber<lvio_ros_msgs::Td> sub_td(nh, "/vins_estimator/td", 100);

    typedef sync_policies::ApproximateTime<lvio_ros_msgs::CorrectData, sensor_msgs::PointCloud, sensor_msgs::PointCloud, lvio_ros_msgs::Td> VisionPolicy;
    Synchronizer<VisionPolicy> sync_vision(VisionPolicy(1000), sub_correct_data, sub_imgs, sub_vision_local_cloud, sub_td);
    sync_vision.registerCallback(boost::bind(&vision_handler, _1, _2, _3, _4));

    ros::spin();
    return 0;
}