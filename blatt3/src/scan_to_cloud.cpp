#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>

ros::Publisher cloud_pub;
int size;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // create a new point cloud
  sensor_msgs::PointCloud cloud;
  cloud.header = scan->header;
  cloud.points.resize(scan->ranges.size());

  // fill the cloud
  for(size_t i = 0; i < scan->ranges.size(); i++)
  {
    cloud.points[i].x = scan->ranges[i] * cos(scan->angle_min + i * scan->angle_increment);
    cloud.points[i].y = scan->ranges[i] * sin(scan->angle_min + i * scan->angle_increment);
  }

  cloud_pub.publish(cloud);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);
  cloud_pub = n.advertise<sensor_msgs::PointCloud>("/cloud", 1);

  ros::spin();
  return 0;
}