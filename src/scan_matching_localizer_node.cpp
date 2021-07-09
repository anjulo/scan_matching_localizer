
// headers in ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pclomp/ndt_omp.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>


class ScanLocalizer
{
    public:
        ScanLocalizer(ros::NodeHandle& nh);        
        ~ScanLocalizer();
        void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
        void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void ICPMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData);
        void NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData);
        
        ros::NodeHandle nh_;
        ros::Subscriber subScan;
        ros::Subscriber subOccupancyGrid;
        ros::Subscriber subInitPose;

        ros::Publisher pubPointCloud;
        ros::Publisher pubStaticMapPointCloud;
        ros::Publisher pubTransformedCloud;
        ros::Publisher pubOdometry;
        


        nav_msgs::Odometry m_odomScan;
    
        pcl::PointCloud<pcl::PointXYZI>::Ptr m_StaticMap_ptr;
        bool bRvizInit;
        Eigen::Matrix4f prev_guess, init_guess;   
};

ScanLocalizer::ScanLocalizer(ros::NodeHandle& nh) : nh_(nh), bRvizInit(false)
{
    subScan = nh_.subscribe("/scan",1, &ScanLocalizer::LaserScanCallback, this);
    subOccupancyGrid = nh_.subscribe("/map",1, &ScanLocalizer::OccupancyGrid2DMapCallback, this);
    subInitPose = nh.subscribe("/initialpose", 1, &ScanLocalizer::InitPoseCallback, this);

    pubPointCloud = nh_.advertise<sensor_msgs::PointCloud2>("/scan_to_pointcloud2", 1, true);
    pubStaticMapPointCloud = nh_.advertise<sensor_msgs::PointCloud2>("/map_to_pointcloud2", 1, true);
    pubTransformedCloud = nh_.advertise<sensor_msgs::PointCloud2>("/registered_points", 1, true);
    pubOdometry = nh_.advertise<nav_msgs::Odometry>("/odom", 1, true);
};

ScanLocalizer::~ScanLocalizer() 
{    
    ROS_INFO("ScanLocalizer destructor.");
}

void ScanLocalizer::InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    Eigen::Matrix3f mat3 = Eigen::Quaternionf(msg.pose.pose.orientation.w, 
                                              msg.pose.pose.orientation.x, 
                                              msg.pose.pose.orientation.y, 
                                              msg.pose.pose.orientation.z).toRotationMatrix();
    prev_guess.block(0,0,3,3) = mat3;
    prev_guess(0,3) = msg.pose.pose.position.x;
    prev_guess(1,3) = msg.pose.pose.position.y;    
    bRvizInit = true;
}

void ScanLocalizer::LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    if(msg->ranges.empty())
    {
        ROS_ERROR("Empty scan data");
        return;
    }

    //convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>) 
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;
    std::vector<float> ranges = msg->ranges;
    std::vector<float> intensities = msg->intensities;

    cloud_in_ptr->is_dense = false;
    cloud_in_ptr->width = ranges.size();
    cloud_in_ptr-> height = 1;
    cloud_in_ptr->points.resize(ranges.size());

    float angle_temp;

    for(int i = 0; i < ranges.size(); i++)
    {
        pcl::PointXYZI pointBuf;
        //#1
        angle_temp = angle_min + i*angle_increment;
        //if (std::isinf(range[i]) == false){
        pointBuf.x = ranges[i]*cos(angle_temp);
        pointBuf.y = ranges[i]*sin(angle_temp);
        pointBuf.z = 0;
        pointBuf.intensity = 0; //msg->intensities[i]; // to be changed
        cloud_in_ptr->points[i] = pointBuf;
    }

    sensor_msgs::PointCloud2 LaserToPointCloud2Msg;
    pcl::toROSMsg(*cloud_in_ptr, LaserToPointCloud2Msg);
    LaserToPointCloud2Msg.header  = msg->header;
    pubPointCloud.publish(LaserToPointCloud2Msg);


    /*Scan matching algorithm*/
    if(cloud_in_ptr->points.empty() || m_StaticMap_ptr->points.empty())
        return;
    

    // You can switch the registration algorithm between ICP and NDT_OMP
    // #2
    // 1. ICP(No multi-thread)
    ICPMatching(cloud_in_ptr, m_StaticMap_ptr); //Too slow
    
    // 2. NDT_OMP(multi-thread)
    //NDTMatching(cloud_in_ptr, m_StaticMap_ptr);
}

void ScanLocalizer::OccupancyGrid2DMapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    // Feel the code to convert the laser scan data to the pointcloud type(pcl::PointCloud<pcl::PointXYZI>)    
    m_StaticMap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr m_StaticMap_ptr;
    for (unsigned int width = 0; width < msg->info.width; width++) {
        for (unsigned int height = 0; height < msg->info.height; height++) {
            // #3
            
            //Convert occupied grid to the x-y coordinates to put the target(pcl::PointCloud<pcl::PointXYZI>)
            if (msg->data[height * msg->info.width + width] > 80) {
                pcl::PointXYZI obstacle;
                //geometry_msgs::Pose obstacle;
                obstacle.x = width * msg->info.resolution + msg->info.resolution / 2 + msg->info.origin.position.x;
                obstacle.y = height * msg->info.resolution + msg->info.resolution / 2 + msg->info.origin.position.y;
                obstacle.z = 0;
                obstacle.intensity = msg->data[height * msg->info.width + width];
                m_StaticMap_ptr->push_back(obstacle);
            }
        }
    }

    sensor_msgs::PointCloud2 StaticMapToPointCloud2Msg;
    pcl::toROSMsg(*m_StaticMap_ptr, StaticMapToPointCloud2Msg);
    StaticMapToPointCloud2Msg.header.frame_id = "map";
    StaticMapToPointCloud2Msg.header.stamp = msg->header.stamp;
    pubStaticMapPointCloud.publish(StaticMapToPointCloud2Msg);

    ROS_INFO("MAP IS LOADED");  
}


void ScanLocalizer::ICPMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData)
{
    // You should initialize the init pose using RVIZ 2D pose estimate tool.
    if (!bRvizInit)
    {
        init_guess = Eigen::Matrix4f::Identity();
    }
    else
    {
        init_guess = prev_guess;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    // set the MaximumIterations, InputSource, InputTarget
    // #4
    
    icp.setInputSource(InputData);
    icp.setInputTarget(TargetData);
    //icp.setMaxCorrespondenceDistance (0.05);
    icp.setMaximumIterations (50);
    //icp.setTransformationEpsilon (1e-8);
    //icp.setEuclideanFitnessEpsilon (1);

    // Run registration algorithm, and put the transformation matrix of previous step.
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*result, init_guess);

    if (icp.hasConverged())
    {
        std::cout << "converged." << std::endl
                << "The score is " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
    }
    else 
    {   
        return;
    }

    init_guess.block<3, 3>(0, 0) = icp.getFinalTransformation().block<3, 3>(0, 0);
    init_guess.block<3, 1>(0, 3) = icp.getFinalTransformation().block<3, 1>(0, 3);

    //publish registered cloud 
    //Convert transformed(registered) pointcloud using ICP algorithm.
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // #5
    // convert pcl pointcloud to the ros msg.  
    pcl::transformPointCloud(*InputData, *final_cloud, init_guess);


    sensor_msgs::PointCloud2 FinalCloudToPointCloud2Msg;
    pcl::toROSMsg(*final_cloud, FinalCloudToPointCloud2Msg); // or pcl::toROSMsg(*result, FinalCloudToPointCloud2Msg);
    FinalCloudToPointCloud2Msg.header.frame_id = "map";
    //FinalCloudToPointCloud2Msg.header.stamp = final_cloud->header.stamp;
    pubTransformedCloud.publish(FinalCloudToPointCloud2Msg);// topic name: "/registered_points"

    //publish Odometry
    //#6
    //Publish the result using nav_msgs/Odometry topic.  
    pubOdometry.publish(FinalCloudToPointCloud2Msg); //topic name: "/odom"

    prev_guess = init_guess;
}

void ScanLocalizer::NDTMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr &InputData, pcl::PointCloud<pcl::PointXYZI>::Ptr &TargetData)
{
    if (!bRvizInit)
    {
        init_guess = Eigen::Matrix4f::Identity();
    }
    else
    {
        init_guess = prev_guess;
    }

    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    //pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt->setInputSource(InputData);
    ndt->setInputTarget(TargetData);    
    ndt->setTransformationEpsilon(0.01); 
    ndt->setMaximumIterations(32); 
    ndt->setResolution(1.0);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    ndt->setNumThreads(5);

    //#7
    // 1. Run registration algorithm(align)
    // 2. Evaluate the registration algorithm using threshold(score)
    // 3. Convert a Final transformation matrix to the init_guess in order to run a align function.
    // 4. Publish registered cloud
    // 5. Publish Odometry
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    ndt->align (*output_cloud, init_guess);

    if (ndt->hasConverged())
    {
        std::cout << "converged." << std::endl
                << "The score is " << ndt->getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:" << std::endl;
        std::cout << ndt->getFinalTransformation() << std::endl;
    }
    else 
    {   
        return;
    }

    init_guess.block<3, 3>(0, 0) = ndt->getFinalTransformation().block<3, 3>(0, 0);
    init_guess.block<3, 1>(0, 3) = ndt->getFinalTransformation().block<3, 1>(0, 3);

    //publish registered cloud 
    //Convert transformed(registered) pointcloud using NDT algorithm.
    pcl::PointCloud<pcl::PointXYZI>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // #5
    //fconvert pcl pointcloud to the ros msg.  
    pcl::transformPointCloud(*InputData, *final_cloud, init_guess);


    sensor_msgs::PointCloud2 FinalCloudToPointCloud2Msg;
    pcl::toROSMsg(*final_cloud, FinalCloudToPointCloud2Msg); // or pcl::toROSMsg(*result, FinalCloudToPointCloud2Msg);
    FinalCloudToPointCloud2Msg.header.frame_id = "map";
    //FinalCloudToPointCloud2Msg.header.stamp = final_cloud->header.stamp;
    pubTransformedCloud.publish(FinalCloudToPointCloud2Msg);// topic name: "/registered_points"

    //publish Odometry
    //#6
    //Publish the result using nav_msgs/Odometry topic.  
    pubOdometry.publish(FinalCloudToPointCloud2Msg); //topic name: "/odom"
    //pubOdometry = nh_.advertise<nav_msgs::Odometry>("/odom", 1, true);

    prev_guess = init_guess;
}

int main(int argc, char** argv)
{    
    // node name initialization
    ros::init(argc, argv, "scan_matching_localizer_node");

    ros::NodeHandle nh;
    ScanLocalizer localizer(nh);

    ros::spin();

    return 0;
}

