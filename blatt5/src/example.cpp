#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

image_transport::Publisher pub;

// shift image 30 pixel to the right
void shift(const cv::Mat& image, cv::Mat& result)
{
  CV_Assert(image.depth() == CV_8U);
  for(int x = 0; x < image.rows; x++)
  {
    for(int y = 30; y < image.cols; y++)
    {
      result.at<uchar>(x, y) = image.at<uchar>(x, y-30);
    }
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(img);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //convert image to grayscale first
  cv::Mat gray, gray2;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  gray2.create(gray.size(),gray.type());

  // example function
  shift(gray, gray2);

  // convert back and publish
  cv::cvtColor(gray2, cv_ptr->image, CV_GRAY2BGR);

  pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shift");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_rect_color", 1, imageCallback);
  pub = it.advertise("image_shift", 1);

  ros::spin();
}
