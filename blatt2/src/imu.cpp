#include <ros/ros.h>
#include <array>
#include <string>

#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <cmath>

ros::Time last_time;

// C++11
std::array<double,3> values; 
std::array<double,3> means;
std::array<double,3> std_devs;
std::array<double,3> sums;
std::array<double,3> squared_sums;

unsigned int cnt;

void init_vars() 
{
    for(int i=0; i<3; ++i) {
        means[i] = 0.0;
        std_devs[i] = 0.0;
        sums[i] = 0.0;
        squared_sums[i] = 0.0;
        cnt = 0;
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Check for bagfile loop
    if(msg->header.stamp < last_time) {
        init_vars(); // reinit
    }
    last_time = msg->header.stamp;


    tf::Quaternion q;     
    tf::quaternionMsgToTF(msg->orientation, q);
    tf::Matrix3x3(q).getRPY(values[0], values[1], values[2]);
    //ROS_INFO("Got IMU-Data: (%lf, %lf, %lf)", values[0], values[1], values[2]);

    // update values (for all three axes)
    cnt ++;
    for(int i=0; i<3; ++i) {

        // increase sums
        sums[i] += values[i];
        squared_sums[i] += values[i]*values[i];

        // compute new mean
        means[i] =  sums[i] / static_cast<double>(cnt);
        
        // standard deviation
        std_devs[i] = sqrt((1.0 / static_cast<double>(cnt-1)) * (squared_sums[i] - (means[i]*sums[i])));
    }

    //ROS_INFO("roll mean: %lf std_dev: %lf", means[0], std_devs[0]);
    //ROS_INFO("pitch mean: %lf std_dev: %lf", means[1], std_devs[1]);
    ROS_INFO("yaw mean: %lf std_dev: %lf", means[2], std_devs[2]);
}

int main(int argc, char **argv)
{
    // init last time
    last_time = ros::Time();
    init_vars();

    ros::init(argc, argv, "imu");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/imu_corrected", 1, imuCallback);
    ros::spin();
    return 0;
}

