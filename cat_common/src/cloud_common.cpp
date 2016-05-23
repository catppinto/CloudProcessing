#include <ros/ros.h>

#include "cat_common/cloud_common.h"

pcl::visualization::PCLVisualizer viewer("t_cloud");
ros::Publisher pub_cylinder;
ros::Publisher pub_centroid;
ros::Publisher pub_objects;
ros::Publisher pub_table;


// ******************* constructors *********************************
 CloudClass::CloudClass(void){
    std::cout << "Hi" << std::endl;
    this->saving_counter=0;
}

CloudClass::CloudClass(std::string filename){

    ROS_INFO(" Trying to load file %s. ", filename.c_str());
    pcl::PCDReader reader;
    //read cloud data
    Cloud_xyz::Ptr cloud1 (new Cloud_xyz());
    if(reader.read(filename, (*cloud1)) !=0)
        ROS_INFO(" Fail to load file");
    else
        ROS_INFO(" Loaded file");
    this->originalCloud= cloud1;
    this->saving_counter=0;
}

CloudClass::CloudClass(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    Cloud2::Ptr cloud (new Cloud2);
    Cloud_xyz::Ptr cloud_plane (new Cloud_xyz ());
    Cloud_xyz::Ptr cloud_cylinder (new Cloud_xyz ());

    //convert to pcl type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    Cloud_xyz::Ptr cloud1 (new Cloud_xyz());
    cloud1 = convertToPointCloud1(cloud);
    this->originalCloud= cloud1;
    this->saving_counter=0;
}

// ******************* get and set *********************************

void CloudClass::loadCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    Cloud2::Ptr cloud (new Cloud2);
    Cloud_xyz::Ptr cloud_plane (new Cloud_xyz ());
    Cloud_xyz::Ptr cloud_cylinder (new Cloud_xyz ());

    //convert to pcl type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    Cloud_xyz::Ptr cloud1 (new Cloud_xyz());
    cloud1 = convertToPointCloud1(cloud);
    this->originalCloud= cloud1;
    this->saving_counter=0;
}

Cloud_xyz::Ptr CloudClass::loadCloud(std::string filename){

   // Read in the cloud data
    pcl::PCDReader reader;
    Cloud_xyz::Ptr cloud (new Cloud_xyz());
    reader.read (filename, *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;

    originalCloud = cloud;
    return cloud;
}

Cloud_xyz::Ptr CloudClass::getCloud(char cloudType) {
    switch(cloudType){
    case 'O':
        return (this->originalCloud);
        break;
    case 'F':
        return (this->filteredCloud);
        break;
    case 'P':
        return (this->planeCloud);
        break;
    case 'C':
        return (this->cylinderCloud);
        break;
    default:
        std::cout << "'p' for processed, 'o' for original cloud";
    }
}

int CloudClass::getSavingCounter(){
(this->saving_counter)++; 
return this->saving_counter;}


Normal::Ptr CloudClass::getSmoothedNormal(){
    return this->smoothedNormals;
}

// ******************* show *********************************

void CloudClass::showOriginalCloud(bool imageFromCamera){

    viewer.removeAllPointClouds();
    viewer.addPointCloud(this->originalCloud, "t_cloud");
    if (imageFromCamera)
        viewer.spinOnce(100);
    else
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
}

void CloudClass::showCloud(bool imageFromCamera, Cloud_xyz::Ptr cloudToShow){

    viewer.removeAllPointClouds();
    viewer.addPointCloud(cloudToShow, "t_cloud");
    if (imageFromCamera)
        viewer.spinOnce(100);
    else
        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
}

// ******************* private functions *********************************

Cloud_xyz::Ptr CloudClass::convertToPointCloud1(Cloud2::Ptr cloud)
{
    Cloud_xyz::Ptr cloud1 (new Cloud_xyz());
    pcl::fromPCLPointCloud2(*cloud, *cloud1);
    return cloud1;
}


// ******************* filters *********************************

void CloudClass::filterZ(int minLimit, int maxLimit){

    Cloud_xyz::Ptr cloud_filtered (new Cloud_xyz());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(this->originalCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(minLimit, maxLimit);
    pass.filter(*cloud_filtered);
    std::cerr << "Filtered Point cloud has: " << cloud_filtered->points.size() << " data points." << std::endl;

    this->filteredCloud = cloud_filtered;
    ROS_INFO("Updated processed cloud");
}

void CloudClass::surfaceSmoothing(){

    pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
    filter.setInputCloud(this->filteredCloud);
    //3 cm radius
    filter.setSearchRadius(0.03);
    // If true, the surface and normal are approximated using a polynomial estimation
        // (if false, only a tangent one).
    filter.setPolynomialFit(true);
    //compute smooth normals
    filter.setComputeNormals(true);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
    filter.setSearchMethod(kdtree);
    filter.process(*smoothedCloud);

   // (this->smoothedNormals) = smoothedCloud;
}

// ******************* computation *********************************

Eigen::Vector4f CloudClass::computeCentroid(Cloud_xyz::Ptr cloud){

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    std::cout << "Centroid : (" << centroid[0] << " , "
                 << centroid[1] << " , "
                 << centroid[2] <<" ) " << std::endl;

//    (this->centroid_pose).pose.position.x = centroid[0];
//    (this->centroid_pose).pose.position.y = centroid[1];
//    (this->centroid_pose).pose.position.z = centroid[2];
//    (this->centroid_pose).pose.orientation.x = 0.0;
//    (this->centroid_pose).pose.orientation.y = 0.0;
//    (this->centroid_pose).pose.orientation.z = 0.0;
//    (this->centroid_pose).pose.orientation.w = 1.0;

    return centroid;
}

// ******************* extractors *********************************

Normal::Ptr CloudClass::normalEstimation(Cloud_xyz::Ptr cloud, int k) {

    ROS_INFO("Computing normals .... ");
    Normal::Ptr normals(new Normal);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setKSearch(k);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(kdtree);
    ne.compute(*normals);


//    Normal::Ptr normals_out (new Normal);
//    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
//    // Specify method for normal estimation
//    norm_est.setNormalEstimationMethod (norm_est.AVERAGE_3D_GRADIENT);
//    // Specify max depth change factor
//    norm_est.setMaxDepthChangeFactor(0.02f);
//    // Specify smoothing area size
//    norm_est.setNormalSmoothingSize(10.0f);
//    // Set the input points
//    norm_est.setInputCloud (cloud);
//    // Estimate the surface normals and
//    // store the result in "normals_out"
//    norm_est.compute (*normals_out);

    return normals;

}

Cloud_xyz::Ptr CloudClass::extractFromModel(Cloud_xyz::Ptr cloud, Normal::Ptr normals, std::string model){

    ROS_INFO("Extracting from model .... ");
    Cloud_xyz::Ptr cloudExtracted(new Cloud_xyz);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;



    seg.setOptimizeCoefficients(true);
    if (model.compare("plane") == 0)
    {
        std::cout << "Here plane model" << std::endl;
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.03);
    }
    else if (model.compare("cylinder") == 0)
    {
        seg.setModelType (pcl::SACMODEL_CYLINDER);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (10000);
        seg.setDistanceThreshold (0.05);
        seg.setRadiusLimits (0, 0.2);
    }
    else if (model.compare("sphere") == 0){
        seg.setModelType(pcl::SACMODEL_SPHERE);
        seg.setMethodType (pcl::SAC_RANSAC);
    }
    else{
         std::cout << "Estou a entrar no else" << std::endl;
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
    }

    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coeffs);

    // extraction
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloudExtracted);

    if (model.compare("plane") == 0)
    {
        this->planeInliers= inliers;
        this->planeCoefficients=coeffs;
    }

    std::cerr << "Filtered Point cloud has: " << cloudExtracted->points.size() << " data points." << std::endl;

    return cloudExtracted;
}

Cloud_xyz::Ptr CloudClass::extractPlane(){

    filterZ(0, 2);

    Cloud_xyz::Ptr c (new Cloud_xyz);
    c = getCloud('F');
    //this->surfaceSmoothing();
    Normal::Ptr n (new Normal);

    n = normalEstimation(c, 50);

    Cloud_xyz::Ptr plane_cloud (new Cloud_xyz);
    plane_cloud= extractFromModel(c, n, "plane");
    this->planeCloud = plane_cloud;
    return plane_cloud;
}

Cloud_xyz::Ptr CloudClass::extractCylinderFromTable(){

    //confirm this
    filterZ(0, 2);

    //this->surfaceSmoothing();
    Normal::Ptr n (new Normal);
    n = normalEstimation(this->filteredCloud, 50);

    std::cout << "Plane!" << std::endl;
    Cloud_xyz::Ptr plane_cloud (new Cloud_xyz);
    plane_cloud = extractFromModel((this->filteredCloud), n, "plane");

    Cloud_xyz::Ptr filtered_cloud (new Cloud_xyz);
    Normal::Ptr filtered_normals (new Normal);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract.setInputCloud(this->filteredCloud);
    extract.setIndices(this->planeInliers);
    extract.setNegative (true);
    extract.filter (*filtered_cloud);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (n);
    extract_normals.setIndices (this->planeInliers);
    extract_normals.filter (*filtered_normals);

    Cloud_xyz::Ptr cylinder_cloud (new Cloud_xyz);

    std::cout << "Cylinder!" << std::endl;
    cylinder_cloud = extractFromModel(filtered_cloud, filtered_normals, "cylinder");

    this->planeCloud = plane_cloud;
    this->cylinderCloud = cylinder_cloud;

    return cylinder_cloud;
}

Cloud_xyz::Ptr CloudClass::extractObjectsOnTable(){

    pcl::ProjectInliers<pcl::PointXYZ> proj_;
    pcl::ConvexHull<pcl::PointXYZ> hull_;
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism_;

    //create cloud without table
    Cloud_xyz::Ptr noTable_cloud (new Cloud_xyz);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(this->filteredCloud);
    extract.setIndices(this->planeInliers);
    extract.setNegative (true);
    extract.filter (*noTable_cloud);

    //project points on table
    Cloud_xyz::Ptr table_projected (new Cloud_xyz);
    proj_.setInputCloud(this->filteredCloud);
    proj_.setIndices(this->planeInliers);
    proj_.setModelCoefficients(this->planeCoefficients);
    proj_.filter(*table_projected);

    //estimate convex hull
    Cloud_xyz::Ptr table_hull (new Cloud_xyz);
    hull_.setInputCloud(table_projected);
    hull_.reconstruct(*table_hull);

    //Get objects on top of table
    pcl::PointIndices::Ptr cloud_obj_indices(new pcl::PointIndices);
    prism_.setInputCloud(noTable_cloud); // acho q se devia tirar a mesa primeiro
    prism_.setInputPlanarHull(table_hull);
    prism_.setHeightLimits(0.0f, 0.1f);
    prism_.segment(*cloud_obj_indices);

    Cloud_xyz::Ptr cloud_objects (new Cloud_xyz);
    pcl::ExtractIndices<pcl::PointXYZ> extract_obj_indices;
    extract_obj_indices.setInputCloud(noTable_cloud);
    extract_obj_indices.setIndices(cloud_obj_indices);
    extract_obj_indices.filter(*cloud_objects);

    if (cloud_objects->points.size() == 0)
        std::cout << "No cloud" << std::endl;
    else
        return cloud_objects;
}

Cloud_xyz::Ptr CloudClass::extractCylinderFromObjects(Cloud_xyz::Ptr cloud){

    Normal::Ptr normals (new Normal);
    normals = normalEstimation(cloud, 20);

    Cloud_xyz::Ptr cylinder_cloud (new Cloud_xyz);

    std::cout << "Cylinder!" << std::endl;
    cylinder_cloud = extractFromModel(cloud, normals, "cylinder");

    this->cylinderCloud = cylinder_cloud;

    return cylinder_cloud;
}

void CloudClass::extractObjectClusters(Cloud_xyz::Ptr cloud){

//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (cloud);

//    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_;
//    std::vector<pcl::PointIndices> object_clusters;
//    cluster_.setInputCloud(cloud);
//    cluster_.setClusterTolerance(0.02);
//    cluster_.setMinClusterSize(100);
//    cluster_.setMaxClusterSize(25000);
//    cluster_.setSearchMethod(tree);
//    cluster_.extract(object_clusters);

//    int j=0;
//    for (std::vector<pcl::PointIndices>::const_iterator it = object_clusters.begin(); it!=object_clusters.end(); ++it)
//    {
//        Cloud_xyz::Ptr cloud_cluster (new Cloud_xyz);
//        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit )
//            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
//        cloud_cluster->width = cloud_cluster->points.size ();
//        cloud_cluster->height = 1;
//        cloud_cluster->is_dense = true;

//        if(cloud_cluster->width > 300)
//        {
//            //try to find cylinder
//            Cloud_xyz::Ptr cylinder (new Cloud_xyz);
//            cylinder = extractCylinderFromObjects(cloud_cluster);


//        }
//        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//        j++;


//    }
}


Eigen::Vector4f CloudClass::getCylinderCentroid(){
    Eigen::Vector4f centroid;
    return (computeCentroid(this->cylinderCloud));
}

// ******************* changeDetectionInFrames *********************************

//http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_(basic)
//http://pointclouds.org/documentation/tutorials/octree_change.php


// ******************* storage *********************************

void CloudClass::storeCloud(Cloud_xyz::Ptr cloudToSave, std::string filename) {

    ROS_INFO("Saving point cloud");
    this->saving_counter++;

    if (cloudToSave->points.empty ())
        std::cout << ("Couldn't store cloud.") << std::endl;
    else
    {
//        filename = "/home/cat/catkin_ws/src/cloudProc/pcdFiles/tablemug_" +
//              boost::to_string(this->savingCounter)
//              + ".pcd";
        ROS_INFO("Saving in : %s data points.", filename.c_str());
        pcl::io::savePCDFileASCII(filename, *(cloudToSave));
    }
}

