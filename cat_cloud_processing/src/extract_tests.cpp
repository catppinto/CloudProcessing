#include <ros/ros.h>
#include <stdlib.h>
#include "cat_common/cloud_common.h"


CloudClass c;
int id_counter;

void customColourVis (boost::shared_ptr<pcl::visualization::PCLVisualizer> my_viewer, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<int> my_colour)
{
    id_counter++;
    std::stringstream frame_id; frame_id << "frame" << id_counter;
    std::string f = frame_id.str();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, my_colour[0], my_colour[1], my_colour[2]);

    my_viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, f);
    my_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, f);
}


std::vector<Cloud_xyz::Ptr> extractCylinderEuclidean(Cloud_xyz::Ptr cloud){


    std::vector<Cloud_xyz::Ptr> cylinderclouds;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    pcl::PCDWriter writer;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    id_counter =0 ;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        Cloud_xyz::Ptr cloud_cluster (new Cloud_xyz());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::vector<int> my_colour;
        my_colour.push_back(rand() % 255);
        my_colour.push_back(rand() % 255);
        my_colour.push_back(rand() % 255);

        if (cloud_cluster->points.size () > 1000)
        {
            std::cout << " ***************************************************************** " << std::endl;
            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

            //checking if is a candidate for cylinder
            pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
            pcl::ExtractIndices<pcl::PointXYZ> extract;

            pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

            pcl::PointCloud<pcl::Normal>::Ptr cloud_cluster_normals (new pcl::PointCloud<pcl::Normal>);

            ne.setSearchMethod (tree);
            ne.setInputCloud (cloud_cluster);
            ne.setKSearch (50);
            ne.compute (*cloud_cluster_normals);

            // Create the segmentation object for cylinder segmentation and set all the parameters
            seg.setOptimizeCoefficients (true);
            seg.setModelType (pcl::SACMODEL_CYLINDER);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight (0.1);
            seg.setMaxIterations (1000);
            seg.setDistanceThreshold (0.01);
            seg.setRadiusLimits (0.0, 0.2);
            seg.setInputCloud (cloud_cluster);
            seg.setInputNormals (cloud_cluster_normals);

            // Obtain the cylinder inliers and coefficients
            seg.segment (*inliers_cylinder, *coefficients_cylinder);
            std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

            // Write the cylinder inliers to disk
            extract.setInputCloud (cloud_cluster);
            extract.setIndices (inliers_cylinder);
            extract.setNegative (false);
            Cloud_xyz::Ptr cloud_cylinder (new Cloud_xyz());
            extract.filter (*cloud_cylinder);
            if (cloud_cylinder->points.empty () || cloud_cylinder->points.size () < 100)
                std::cerr << "Can't find the cylindrical component." << std::endl;
            else
            {
                customColourVis (viewer, cloud_cylinder, my_colour);
                cylinderclouds.push_back(cloud_cluster);
                std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
            }
        }

        j++;
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}


// ******************* MAIN *********************************

int main(int argc, char** argv)
{

    if(argc > 1)
    {
        std::string filename = argv[0];
        c.loadCloud(filename);
    }
    else
        c.loadCloud("/home/apirespi/my_workspace/src/cat_cloud_processing/pcd_files_storage/herbKinect2pointcloud_1.pcd");

    Cloud_xyz::Ptr objects (new Cloud_xyz);
    c.extractPlane();
    Cloud_xyz::Ptr planeCloud(new Cloud_xyz);
    planeCloud = c.getCloud('P');
    c.showCloud(false, planeCloud);
	
    //objects = c.extractObjectsOnTable();

    //std::vector<Cloud_xyz::Ptr> cloudclusters;
    //cloudclusters = extractCylinderEuclidean(objects);

//    int i=0;
//    for (i=0; i< cloudclusters.size(); i++)
//        std::cout << "CLOUD #" << i << " nr of points : " << (cloudclusters[i])->points.size() << std::endl;

}

