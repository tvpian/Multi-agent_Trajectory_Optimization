#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Eigen>
#include <math.h>
#include <random>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <mission.hpp>

using namespace std;

class MapGenerator
{
public:
    virtual void RandomMapGenerate(const TrajPlanning::Mission &mission) = 0;
    
    void pubSensedPoints()
    {
        pcl::toROSMsg(cloudMap, globalMap_pcd);
        globalMap_pcd.header.frame_id = "world";
        all_map_pub.publish(globalMap_pcd);
    }

    ros::Publisher all_map_pub;
    int obs_num;
    double margin;
    double x_min, y_min, z_min, x_max, y_max, z_max;
    double r_min, r_max, h_min, h_max, resolution;

protected:
    pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;

    uniform_real_distribution<double> rand_x;
    uniform_real_distribution<double> rand_y;
    uniform_real_distribution<double> rand_w;
    uniform_real_distribution<double> rand_h;

    vector<double> _state;

    sensor_msgs::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
};