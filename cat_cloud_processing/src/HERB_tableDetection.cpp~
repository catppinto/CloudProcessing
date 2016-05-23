#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>
#include <tf/transform_listener.h>

#include "cat_common/cloud_common.h"


//https://github.com/wg-perception/tabletop/blob/master/src/table/TableDetector.cpp 
//check link above

ros::Publisher pub_table;

CloudClass cloud;

int savingCounter;

/**
 * If the equation of the plane is ax+by+cz+d=0, the pose (R,t) is such that it takes the horizontal plane (z=0)
 * to the current equation
 */
void getPlaneTransform(const cv::Vec4f& plane_coefficients, cv::Matx33f& rotation, cv::Vec3f& translation)
{
  double a = plane_coefficients[0], b = plane_coefficients[1], c = plane_coefficients[2], d = plane_coefficients[3];
  // assume plane coefficients are normalized
  translation = cv::Vec3f(-a * d, -b * d, -c * d);
  cv::Vec3f z(a, b, c);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  cv::Vec3f x(1, 0, 0);
  if (fabs(z.dot(x)) > 1.0 - 1.0e-4)
    x = cv::Vec3f(0, 1, 0);
  cv::Vec3f y = z.cross(x);
  x = y.cross(z);
  x = x / norm(x);
  y = y / norm(y);

  rotation = cv::Matx33f(x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
}



void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

	std::cout << "in cloud_cb" << std::endl;
	cloud.loadCloud(cloud_msg);

	Cloud_xyz::Ptr objects (new Cloud_xyz);
	cloud.extractPlane();

	Cloud_xyz::Ptr plane (new Cloud_xyz);
	plane = cloud.getCloud('P');

	ROS_INFO("after plane extraction ... ");

	sensor_msgs::PointCloud2 cloud_table_output;
	pcl::toROSMsg(*plane, cloud_table_output);
	pub_table.publish(cloud_table_output);

	pcl::PointIndices::Ptr planeIndices ( new pcl::PointIndices);
	planeIndices = cloud.getPlaneIndices();
	pcl::ModelCoefficients::Ptr modelCoeffs (new pcl::ModelCoefficients);
	modelCoeffs = cloud.getPlaneModelCoefficients();

	std::cout << "Model Coefs : " << std::endl;
	std::cout << *modelCoeffs << std::endl;




}

bool storePlaneCloudFromService(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res){

	savingCounter +=1;
    ROS_INFO("Saving plane cloud");

    std::string filename;
    Cloud_xyz::Ptr planecloud (new Cloud_xyz);
    planecloud = cloud.getCloud('P');
    Cloud_xyz::Ptr oricloud (new Cloud_xyz);
    oricloud = cloud.getCloud('O');
    if (planecloud->points.empty ())
        ROS_INFO("Couldn't store cloud.");
    else if(oricloud->points.empty ())
        ROS_INFO("Couldn't store cloud.");
    else
    {
	int counter = cloud.getSavingCounter();
        filename = "/home/apirespi/my_workspace/src/cat_cloud_processing/pcd_files_storage/planeClouds/planecloud_" +
              boost::to_string(savingCounter)
              + ".pcd";
        ROS_INFO("Saving in : %s data points.", filename.c_str());
        pcl::io::savePCDFileASCII(filename, *planecloud);

	// save original
	 filename = "/home/apirespi/my_workspace/src/cat_cloud_processing/pcd_files_storage/planeClouds/originalcloud_" +
		      boost::to_string(savingCounter)
              + ".pcd";
        ROS_INFO("Saving in : %s data points.", filename.c_str());
        pcl::io::savePCDFileASCII(filename, *oricloud);
    }
    return true;
}




// ******************* MAIN *********************************

int main(int argc, char** argv){

    savingCounter = 0;

    if(argc<=2)
    {
        // initializing rosNode and Loading Image from camera
        ros::init(argc, argv, "cloudProc");
        ros::NodeHandle nh;
        ros::Subscriber sub;

        if(argc == 1)
            sub = nh.subscribe("head/kinect2/qhd/points", 1, cloud_cb);
        else
        {
            std::cout << "Entrei no sitio certo " << std::endl;
            std::string subscription_node = argv[1];
            std::cout << subscription_node << std::endl;
            sub = nh.subscribe(subscription_node, 1, cloud_cb);
        }

        ros::ServiceServer service = nh.advertiseService("storePlaneCloud", storePlaneCloudFromService);

        pub_table = nh.advertise<sensor_msgs::PointCloud2>("planeCloud",1);

        ros::spin();

    }
    else
    {
        //mudar isto
        CloudClass cloud(argv[2]);

        Cloud_xyz::Ptr plane (new Cloud_xyz);
        cloud.extractPlane();
	plane = cloud.getCloud('P');

        ROS_INFO("after plane extraction ... ");

        pcl::visualization::PCLVisualizer viewer("t_cloud"); //tirar daqui
        viewer.addPointCloud(cloud.getCloud('F'), "cenas");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(plane, 255, 0, 0);
        viewer.addPointCloud(plane, colorHandler, "coisas");
        while(!viewer.wasStopped())
                viewer.spinOnce(100);

        ROS_INFO("clouds shown ... ");

    }
}

