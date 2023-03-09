#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
float sensor_data=-1;

void sensorCallback(const std_msgs::Float64::ConstPtr& msg){
  sensor_data = msg->data;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "capra_sensor_mapping");
  ros::NodeHandle nh;

  sensor_msgs::PointCloud2 output;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width    = 1;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.resize (cloud.width * cloud.height);
  cloud.header.frame_id = "map";

  tf::TransformListener listener;

  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  ros::Subscriber sub = nh.subscribe("sensor_input", 1, sensorCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(1000.0);
  while (ros::ok()){

    tf::StampedTransform transform;
    try{
      listener.lookupTransform("map", "base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    pcl::PointXYZI point;

    point.x=transform.getOrigin().x();
    point.y=transform.getOrigin().y();
    point.z=transform.getOrigin().z();
    point.intensity=sensor_data;

    cloud.insert(cloud.end(), point);
 
    pcl::toROSMsg(cloud, output);
    pub.publish (output);
    rate.sleep();
  }
}