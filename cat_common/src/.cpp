#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "cat_common/cloud_common.h"


void extractObjectClusters(Cloud_xyz::Ptr cloud){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_;
    std::vector<pcl::PointIndices> object_clusters;
    cluster_.setInputCloud(cloud);
    cluster_.setClusterTolerance(0.02);
    cluster_.setMinClusterSize(100);
    cluster_.setMaxClusterSize(25000);
    cluster_.setSearchMethod(tree);
    cluster_.extract(object_clusters);

    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = object_clusters.begin (); it != object_clusters.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "/home/cat/catkin_ws/src/biovision/cat_common/tests_pcd/cloud_cluster_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
      j++;
    }
}

int main(int argc, char** argv){

    CloudClass c;
    Cloud_xyz::Ptr cloud;
    if(argc > 1){
        std::string filename = argv[0];
        cloud = c.loadCloud(filename);
    }
    else{
        cloud = c.loadCloud("/home/cat/catkin_ws/src/tutorials/pcd_files/tablemug_1.pcd");
    }

    c.extractPlane();

    Cloud_xyz::Ptr objects (new Cloud_xyz);
    objects = c.extractObjectsOnTable();

    extractObjectClusters(objects);
    std::cout << " Extracted clusters " << std::endl;
}
