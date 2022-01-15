
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <map>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher cloud_pub;

void
CloudCallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr remove_NaN_cloud {new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr affine_cloud {new pcl::PointCloud<pcl::PointXYZ>};
    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *cloud);
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *remove_NaN_cloud, mapping);
    pcl::removeNaNFromPointCloud(*cloud, *affine_cloud, mapping);
    //アフィンの生成
    Eigen::Matrix4f affine;
    //x方向1m, y方向2m, x軸回り-90度
    affine <<   1, 0, 0, 1,
                0, 0,-1, 2,
                0, 1, 0, 0,
                0, 0, 0, 1;
    Eigen::Vector4f input_point;
    Eigen::Vector4f output_point;

    // TF
    for(size_t i=0;i<remove_NaN_cloud->points.size();i++){
        //点群のベクトル化
        input_point(0) = remove_NaN_cloud->points[i].x;
        input_point(1) = remove_NaN_cloud->points[i].y;
        input_point(2) = remove_NaN_cloud->points[i].z;
        input_point(3) = 1;
        //変換
        output_point = affine * input_point;
        //ベクトルの点群化
        affine_cloud->points[i].x = output_point(0);
        affine_cloud->points[i].y = output_point(1);
        affine_cloud->points[i].z = output_point(2);
    }
    sensor_msgs::PointCloud2 affine_cloud_pub;
	pcl::toROSMsg(*affine_cloud, affine_cloud_pub);
    cloud_pub.publish(affine_cloud_pub);
}


int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pointcloud_transform");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber cloud_sub = nh.subscribe ("/cloud", 1, CloudCallback);

    // Create a ROS publisher for the output point cloud
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/affine_cloud", 10);
    // Spin
    ros::spin ();
}