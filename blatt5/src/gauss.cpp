#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

image_transport::Publisher pub;
int kernel_size = 10;

void imageCallback(const sensor_msgs::ImageConstPtr &img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //convert image to grayscale first
  cv::Mat gray, gray2;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  gray2.create(gray.size(), gray.type());

  for (int i = 1; i < kernel_size; i = i + 2)
  {
    cv::GaussianBlur(gray, gray2, cv::Size(i, i), 0, 0);
  }

  // convert back and publish
  cv::cvtColor(gray2, cv_ptr->image, CV_GRAY2BGR);

  pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gauss");

  ros::NodeHandle n("~");
  if (n.getParam("kernel_size", kernel_size))
  {
    ROS_INFO_STREAM("Got param 'kernel_size': " << kernel_size);
    if (kernel_size < 2)
    {
      ROS_ERROR("Invalid Kernel size");
      ros::shutdown();
    }
  }
  else
  {
    ROS_ERROR_STREAM("Failed to get param 'kernel_size'. Setting default val " << kernel_size);
  }

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_rect_color", 1, imageCallback);
  pub = it.advertise("gaussianified", 1);

  ros::spin();
}
