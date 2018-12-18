
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <angles/angles.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <kurt_msgs/Encoder.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

		
const double axle_length= 0.28; // Kurt2's axle length in meter
const double r_x = 0.05;  // robot control noise x direction
const double r_y = 0.05;  // robot control noise y direction 
const double r_theta = 5.0 * M_PI / 180.0;  // robot control noise heading
const double sigma_z = 0.004; // variance of imu

double roll, pitch, yaw, first_yaw;
bool first_yaw_set;

// The magical correction factor for indoor Kurt2
const double friction_loss_multiplier= 0.69;

// Declare our state
Eigen::Vector3f mu_t;
Eigen::Matrix3f Sigma_t;

// This converts from Kurt2 motor encoder tics to meter
inline double encoder2meter (unsigned int enc)
{
	return static_cast<double>(0.379/21950.0) * enc;
}

// reset odometry for usage of bagfile with loop
void reset_odom()
{
  mu_t = Eigen::Vector3f::Zero ();
  Sigma_t = Eigen::Matrix3f::Zero ();
  first_yaw = 0.0;
  yaw = 0.0;
  first_yaw_set = false;
}

// Return ROS-time since last call
ros::Duration update_timer()
{
	static ros::Time last_time(0.0);

	ros::Time now= ros::Time::now();
	ros::Duration delta= now-last_time;
	last_time= now;

	return delta;
}

void ekfCallback (const sensor_msgs::Imu::ConstPtr& imu, const kurt_msgs::Encoder::ConstPtr& msg)
{
	if( update_timer () < ros::Duration (-1.0) )
  {
		ROS_WARN ("detected loop, resetting odometry");
		reset_odom ();
		return;
	}
  static tf::TransformBroadcaster br;

  // retrive distance for right and left wheel
	const double left =  encoder2meter (msg->left);
	const double right = encoder2meter (msg->right);

  // compute travel distance of center and change in orientation (slide 20 ff.)
	const double dist = static_cast<double>(left + right) * .5;
	double delta_theta = static_cast<double>(right - left) / axle_length;
  // use Kurt2 specific parameter for turning
  delta_theta *= friction_loss_multiplier;
  // normalize to [-M_PI, M_PI] (so that it fits the data from the imu)
  delta_theta = angles::normalize_angle (delta_theta);

  // implement simple forward kinematics
  Eigen::Vector3f forward_k;
  forward_k << dist * cos (mu_t (2) + delta_theta * .5),
            dist * sin (mu_t (2) + delta_theta * .5),
            delta_theta;

  // simple update via forward kinematics
  //mu_t += forward_k;

  // compute Jacobian F_t
  Eigen::Matrix3f F_t = Eigen::Matrix3f::Identity ();
  F_t (0,2) = -dist * sin (mu_t (2) + delta_theta * .5);
  F_t (1,2) = dist * cos (mu_t (2) + delta_theta * .5);

  Eigen::Matrix3f Sigma_u = Eigen::Matrix3f::Zero ();
  Sigma_u (0, 0) = r_x * r_x;
  Sigma_u (1, 1) = r_y * r_y;
  Sigma_u (2, 2) = r_theta * r_theta; 


  // compute a-priori estimaton from current mean and forward kinematics
  Eigen::Vector3f mu_bar_t = mu_t + forward_k;
  // compute a-priori covariance
  Eigen::Matrix3f Sigma_bar_t = F_t * Sigma_t * F_t.transpose () + Sigma_u;

  // compute the Kalman gain
  Eigen::MatrixXf H_t (1, 3);
  H_t << 0.0, 0.0, 1.0;
  Eigen::MatrixXf Sigma_z (1, 1);
  Sigma_z << sigma_z * sigma_z;
  Eigen::MatrixXf K (3, 1);
  K = Sigma_bar_t * H_t.transpose () * (H_t * Sigma_bar_t * H_t.transpose () + Sigma_z).inverse ();

  // retrieve theta angle provided by imu
  tf::Quaternion q;
  tf::quaternionMsgToTF (imu->orientation, q);
  tf::Matrix3x3 (q).getRPY (roll, pitch, yaw);
  double imu_delta_theta;
  if (first_yaw_set)
  {
    imu_delta_theta = angles::shortest_angular_distance (first_yaw, yaw);
//    ROS_INFO ("%lf, %lf", imu_delta_theta, mu_bar_t (2));
  }
  else
  {
    imu_delta_theta = mu_bar_t (2);
  }


  mu_t = mu_bar_t + K * (imu_delta_theta - mu_bar_t (2));

  // normalize the angle
  mu_t (2) = angles::normalize_angle (mu_t (2));

  Sigma_t = (Eigen::Matrix3f::Identity () - K * H_t) * Sigma_bar_t;

  // set translation for odometry message
  tf::Vector3 translation (mu_t (0), mu_t (1), 0.0);
  // set rotation for odometry message
  tf::Quaternion rotation;
  rotation.setRPY (0.0, 0.0, mu_t (2));

  // store last imu measurement
  if (!first_yaw_set)
  {
    first_yaw = yaw;
    first_yaw_set = true;
  }


  // create a tf::Transform to broadcast the odometry
  tf::Transform transform;
  transform.setOrigin (translation);
  transform.setRotation (rotation);

  // broadcast the odometry
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "/odom_combined", msg->header.frame_id));

}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "kurt_ekf");

  ros::NodeHandle n;

  // call reset to initialize mean and covariance
  reset_odom ();

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(n,"/imu",1);
  message_filters::Subscriber<kurt_msgs::Encoder> enc_sub (n,"/encoder", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, kurt_msgs::Encoder> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, enc_sub);
  sync.registerCallback(boost::bind(&ekfCallback, _1, _2));
  
  ros::spin();
  return 0;
}
