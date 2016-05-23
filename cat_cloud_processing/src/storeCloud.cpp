#include <ros/ros.h>
#include <iostream>
#include <string.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <sstream>

#include <std_srvs/Empty.h>

typedef pcl::PointCloud< pcl::PointXYZ > Cloud_xyz;
typedef pcl::PCLPointCloud2 Cloud2;


Cloud_xyz::Ptr cloud_xyz1 (new Cloud_xyz());
int counter;
pcl::visualization::PCLVisualizer viewer("t_cloud");

void showCloud(){
    std::string user_input;
    int counter = 0;

    viewer.removeAllPointClouds();
    viewer.addPointCloud(cloud_xyz1, "t_cloud");
    viewer.spinOnce(100);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    Cloud2::Ptr cloud_xyz2 (new Cloud2);

    //convert to pcl type
    pcl_conversions::toPCL(*cloud_msg, *cloud_xyz2);

    std::cout << " Showing Clouds ... " << std::endl;

    pcl::fromPCLPointCloud2(*cloud_xyz2, *cloud_xyz1);

    showCloud();

}

bool store(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res)
{

    ROS_INFO("Saving point cloud");
    counter ++;

    std::string filename = "";
    if (cloud_xyz1->points.empty ())
        ROS_INFO("Couldn't store cloud.");
    else
    {

        filename = "/home/apirespi/my_workspace/src/cat_cloud_processing/pcd_files_storage/herbKinect2pointcloud_" +
              static_cast<std::ostringstream*>( &(std::ostringstream() << counter) )->str()
              + ".pcd";
        ROS_INFO("Saving in : %s data points.", filename.c_str());
        pcl::io::savePCDFileASCII(filename, *cloud_xyz1);
    }
    return true;
}

int main(int argc, char** argv){

    counter = 0;
    ros::init(argc, argv, "storeCloud");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/head/kinect2/qhd/points", 1, cloud_cb);
    ros::ServiceServer service = nh.advertiseService("storeCloud", store);

    ros::spin();

}
