#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>
#include <string>

ros::Publisher pub;
float sensor_data=-1;

float grid_size = 1;
float gain = 0.1;

void sensorCallback(const std_msgs::Float64::ConstPtr& msg){
  sensor_data = msg->data;
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "capra_sensor_mapping");
  ros::NodeHandle nh;

  std::string reference_frame = "map";
  std::string sensor_frame = "base_link";

  if(nh.getParam("reference_frame", reference_frame)==false){
    ROS_WARN("Missing reference_frame Param, assuming map");
  }
  if(nh.getParam("sensor_frame", sensor_frame)==false){
    ROS_WARN("Missing sensor_frame Param, assuming base_link");
  }
  if(nh.getParam("filter_gain", gain)==false){
    ROS_WARN("Missing filter_gain Param, assuming 0.1");
  }
  if(nh.getParam("grid_size", grid_size)==false){
    ROS_WARN("Missing grid_size Param, assuming 1");
  }

  sensor_msgs::PointCloud2 output;

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.width    = 1;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.resize (cloud.width * cloud.height);
  cloud.header.frame_id = reference_frame;


  tf::TransformListener listener;

  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  ros::Subscriber sub = nh.subscribe("sensor_input", 1, sensorCallback);

  ros::Duration(1.0).sleep();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(100.0);
  while (ros::ok()){

    tf::StampedTransform transform;
    try{
      listener.lookupTransform(reference_frame, sensor_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }


    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud.makeShared());

    // set the position of the point to search for
    pcl::PointXYZI search_point;
    search_point.x = grid_size * round(transform.getOrigin().x()/grid_size);
    search_point.y = grid_size * round(transform.getOrigin().y()/grid_size);
    search_point.z = grid_size * round(transform.getOrigin().z()/grid_size);

    // perform the nearest neighbor search
    int k = 1; // search for the nearest point
    std::vector<int> indices(k);
    std::vector<float> distances(k);
    kdtree.nearestKSearch(search_point, k, indices, distances);


    if(sensor_data >= 0){
      if(distances[0] >= grid_size){
        pcl::PointXYZI point;
        point.x = grid_size * round(transform.getOrigin().x()/grid_size);
        point.y = grid_size * round(transform.getOrigin().y()/grid_size);
        point.z = grid_size * round(transform.getOrigin().z()/grid_size);
        point.intensity=sensor_data;
        cloud.insert(cloud.end(), point);
      }
      else{
        cloud.at(indices[0]).intensity =  cloud.at(indices[0]).intensity  + gain * (sensor_data - cloud.at(indices[0]).intensity);
      }
    }
    
    pcl::toROSMsg(cloud, output);
    pub.publish (output);
    rate.sleep();
  }
}