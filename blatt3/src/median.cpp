#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

ros::Publisher laser_pub;
int size;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // copy scan, so we can modify it
  sensor_msgs::LaserScan median(*scan);
  unsigned int offset = size/2;
  // define window
  std::vector<float> window(size);
  // iterate until window is at the end of the array
  for(int i=0; i<scan->ranges.size()-size; ++i) {
    // compute median
    window.assign(&scan->ranges[i], &scan->ranges[i+size]);
    std::nth_element(window.begin(), window.begin()+ offset, window.end());
    median.ranges[i+offset] = window[offset];
  }

  // Optional: Border cases (by reflecting the values)
  std::vector<float> border_values(size+1);
  // low border
  int scan_index = -offset;
  for(int i=0; i<size+1; ++i) {
    border_values[i] = scan->ranges[abs(scan_index++)];
  }
  for(int i=0; i<offset; ++i) {
    // compute median
    window.assign(&border_values[i], &border_values[i+size]);
    std::nth_element(window.begin(), window.begin()+ offset, window.end());
    median.ranges[i] = window[offset];
  }
  // high border
  scan_index = scan->ranges.size()-offset;
  for(int i=0; i<size+1; ++i) {
    border_values[i] = scan->ranges[scan_index];
    if(scan_index == scan->ranges.size()-1) {
      scan_index--;
    } else {
      scan_index++;
    }
  }
  for(int i=scan->ranges.size()-offset; i<scan->ranges.size(); ++i) {
    // compute median
    window.assign(&border_values[i], &border_values[i+size]);
    std::nth_element(window.begin(), window.begin()+ offset, window.end());
    median.ranges[i] = window[offset];
  }


  laser_pub.publish(median);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "median");
  ros::NodeHandle n;

  ros::NodeHandle nh("~");
  nh.getParam("size", size);
  if(size <= 0) {
    size = 5;
    ROS_INFO("using default size %d", size);
  }
  ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);
  laser_pub = n.advertise<sensor_msgs::LaserScan>("/scan_median", 1);

  ros::spin();
  return 0;
}
