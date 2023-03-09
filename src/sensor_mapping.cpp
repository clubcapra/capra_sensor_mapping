#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "capra_sensor_mapping");
  ros::NodeHandle nh;

  sensor_msgs::PointCloud2 output;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(10.0);
  while (ros::ok()){
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.resize (cloud.width * cloud.height);

    for(int i = 0; i<5; i++){
      cloud.points.at(i).x=i;
      cloud.points.at(i).z=0;
      cloud.points.at(i).y=0;
      cloud.points.at(i).intensity=i;
    }
    cloud.header.frame_id = "map";
    pcl::toROSMsg(cloud, output);
    pub.publish (output);
    // Spin
    rate.sleep();
  }
}