#include <ros/ros.h>
//#include <iostream>
//#include <string>

#include "cat_common/cloud_common.h"

//#include <pcl/surface/mls.h>

//pcl::visualization::PCLVisualizer viewer("t_cloud");
ros::Publisher pub_cylinder;
ros::Publisher pub_centroid;
ros::Publisher pub_objects;
ros::Publisher pub_table;

CloudClass cloud;


void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    std::cout << "in cloud_cb" << std::endl;
    cloud.loadCloud(cloud_msg);

    Cloud_xyz::Ptr objects (new Cloud_xyz);
    cloud.extractPlane();

    objects = cloud.extractObjectsOnTable();
    Cloud_xyz::Ptr cylinder (new Cloud_xyz);
    cylinder = cloud.extractCylinderFromObjects(objects);

    Eigen::Vector4f centroid;
    centroid = cloud.getCylinderCentroid();

    Cloud_xyz::Ptr plane (new Cloud_xyz);
    plane = cloud.getCloud('P');

    ROS_INFO("after plane extraction ... ");

//    viewer.removeAllPointClouds();
//    viewer.addPointCloud(cloud.getCloud('F'), "cenas");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(plane, 255, 0, 0);
//    viewer.addPointCloud(plane, colorHandler, "coisas");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler2(cylinder, 0, 255, 0);
//    viewer.addPointCloud(cylinder, colorHandler2, "maiscoisas");
//    viewer.spinOnce(100);

    ROS_INFO("clouds shown ... ");

    // create container
    sensor_msgs::PointCloud2 cloud_cylinder_output;
    pcl::toROSMsg(*cylinder, cloud_cylinder_output);
    pub_cylinder.publish(cloud_cylinder_output);

    sensor_msgs::PointCloud2 cloud_obj_output;
    pcl::toROSMsg(*objects, cloud_obj_output);
    pub_objects.publish(cloud_obj_output);

    sensor_msgs::PointCloud2 cloud_table_output;
    pcl::toROSMsg(*plane, cloud_table_output);
    pub_table.publish(cloud_table_output);

    geometry_msgs::PoseStamped centroid_pose;
    centroid_pose.pose.position.x = centroid[0];
    centroid_pose.pose.position.y = centroid[1];
    centroid_pose.pose.position.z = centroid[2];
    centroid_pose.pose.orientation.x = 0.0f;
    centroid_pose.pose.orientation.y = 0.0f;
    centroid_pose.pose.orientation.z = 0.0f;
    centroid_pose.pose.orientation.w = 1.0f;
    centroid_pose.header.frame_id = "camera_depth_optical_frame";
    //centroid_pose.header.frame_id = "second_camera_depth_optical_frame";
    centroid_pose.header.stamp = ros::Time::now();
    pub_centroid.publish(centroid_pose);


}

bool storeCloudFromService(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){

    ROS_INFO("Saving point cloud");

    std::string filename = "";
    Cloud_xyz::Ptr this_cloud (new Cloud_xyz);
    this_cloud = cloud.getCloud('O');
    if (this_cloud->points.empty ())
        ROS_INFO("Couldn't store cloud.");
    else
    {

        filename = "/home/cat/catkin_ws/src/biovision/cloud_processing/pcd_files/cloud_" +
              boost::to_string(cloud.getSavingCounter())
              + ".pcd";
        ROS_INFO("Saving in : %s data points.", filename.c_str());
        pcl::io::savePCDFileASCII(filename, *this_cloud);
    }
    return true;
}




// ******************* MAIN *********************************

int main(int argc, char** argv){


    if(argc<=2)
    {
        // initializing rosNode and Loading Image from camera
        ros::init(argc, argv, "cloudProc");
        ros::NodeHandle nh;
        ros::Subscriber sub;

        if(argc == 1)
            sub = nh.subscribe("camera/depth/points", 1, cloud_cb);
        else
        {
            std::cout << "Entrei no sitio certo " << std::endl;
            std::string subscription_node = argv[1];
            std::cout << subscription_node << std::endl;
            sub = nh.subscribe(subscription_node, 1, cloud_cb);
        }

        ros::ServiceServer service = nh.advertiseService("store_cloud", storeCloudFromService);

        pub_cylinder = nh.advertise<sensor_msgs::PointCloud2>("cylinder",1);
        pub_objects = nh.advertise<sensor_msgs::PointCloud2>("objects",1);
        pub_centroid = nh.advertise<geometry_msgs::PoseStamped>("centroid",1);
        pub_table = nh.advertise<sensor_msgs::PointCloud2>("plane",1);

        ros::spin();

    }
    else
    {
        //mudar isto
        CloudClass cloud(argv[2]);

        Cloud_xyz::Ptr objects (new Cloud_xyz);
        cloud.extractPlane();
        objects = cloud.extractObjectsOnTable();
        Cloud_xyz::Ptr cylinder (new Cloud_xyz);
        cylinder = cloud.extractCylinderFromObjects(objects);

        Eigen::Vector4f centroid;
        centroid = cloud.getCylinderCentroid();


        ROS_INFO("after plane extraction ... ");

        pcl::visualization::PCLVisualizer viewer("t_cloud"); //tirar daqui
        viewer.addPointCloud(cloud.getCloud('F'), "cenas");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(objects, 255, 0, 0);
        viewer.addPointCloud(objects, colorHandler, "coisas");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler2(cylinder, 0, 255, 0);
        viewer.addPointCloud(cylinder, colorHandler2, "maiscoisas");
        while(!viewer.wasStopped())
                viewer.spinOnce(100);

        ROS_INFO("clouds shown ... ");

//        ROS_INFO("Saving cloud ...");
//        int counter= cloud.getSavingCounter();
//        std::string filename = "/home/cat/catkin_ws/src/cloudProc/pcdFiles/tablemug_" +
//                     boost::to_string(counter)
//                     + ".pcd";
//        cloud.storeCloud(c, filename);
//        ROS_INFO("Cloud stored");
    }
}

