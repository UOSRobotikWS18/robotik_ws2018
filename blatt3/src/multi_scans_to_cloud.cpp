#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <cmath>

ros::Publisher cloud_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan1,const sensor_msgs::LaserScan::ConstPtr& scan2)
{
  static tf::TransformListener tflistener;

  // create a new point cloud
  sensor_msgs::PointCloud cloud, cloud2, transformed_cloud;
  
  cloud.header = scan1->header;
  cloud.points.resize(scan1->ranges.size() + scan2->ranges.size()); // init for both clouds
  for(size_t i = 0; i < scan1->ranges.size(); i++)
  {
    cloud.points[i].x = scan1->ranges[i] * cos(scan1->angle_min + i * scan1->angle_increment);
    cloud.points[i].y = scan1->ranges[i] * sin(scan1->angle_min + i * scan1->angle_increment);
  }

  // create the 2nd point cloud
  cloud2.header = scan2->header;
  cloud2.points.resize(scan2->ranges.size());
  for(size_t i = 0; i < scan2->ranges.size(); i++)
  {
    cloud2.points[i].x = scan2->ranges[i] * cos(scan2->angle_min + i * scan2->angle_increment);
    cloud2.points[i].y = scan2->ranges[i] * sin(scan2->angle_min + i * scan2->angle_increment);
  }

  // transform
  try
  {
    tflistener.transformPointCloud("base_laser_link", cloud2, transformed_cloud);
  } catch(tf::TransformException ex)
  { }

  // write transformed cloud into the first one
  for(size_t i = 0; i < transformed_cloud.points.size(); i++)
  {
    cloud.points[i+scan1->ranges.size()].x = transformed_cloud.points[i].x;
    cloud.points[i+scan1->ranges.size()].y = transformed_cloud.points[i].y;
  }

  cloud_pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;

  message_filters::Subscriber<sensor_msgs::LaserScan> scan1_sub(n,"/scan",1);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan2_sub(n,"/laserscan",1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), scan1_sub, scan2_sub);
  sync.registerCallback(boost::bind(&laserCallback, _1, _2));
  cloud_pub = n.advertise<sensor_msgs::PointCloud>("/cloud", 1);

  ros::spin();
  return 0;
}