// @author: Sven Albrecht
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <visualization_msgs/Marker.h>
#include <angles/angles.h>

ros::Publisher line_pub;
ros::Publisher post_cloud_pub;
ros::Publisher pre_cloud_pub;

double max_dist2;
double theta_threshold;
bool use_icp, use_imr, use_idc;
tf::Transform global_transform;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;

typedef std::pair<geometry_msgs::Point32, geometry_msgs::Point32> CorrPair;
typedef std::vector<CorrPair> CorrVec;

laser_geometry::LaserProjection projector_;

float
sqrdDist (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) +
    (p1.y - p2.y) * (p1.y - p2.y);
}

void
drawLines (const CorrVec &point_correspondences,
    visualization_msgs::Marker &lines)
{
  CorrVec::const_iterator it = point_correspondences.begin ();
  lines.points.reserve (point_correspondences.size () * 2);
  geometry_msgs::Point p;
  while (it != point_correspondences.end ())
  {
    p.x = it->first.x;
    p.y = it->first.y;
    lines.points.push_back (p);
    p.x = it->second.x;
    p.y = it->second.y;
    lines.points.push_back (p);
    it++;
  }
}

void
calcIMRCorrespondences (const sensor_msgs::PointCloud::ConstPtr &scan_cloud,
    const sensor_msgs::PointCloud::ConstPtr &model_cloud,
    CorrVec &point_correspondences)
{
  point_correspondences.clear ();
  point_correspondences.reserve (scan_cloud->points.size ());
  
  std::vector<geometry_msgs::Point32>::const_iterator scan_it, model_it,
    best_it;
 
  scan_it = scan_cloud->points.begin ();
  double min_dist2;
  double p_r, m_r;
  double p_theta, m_theta;
  double dist2;
  bool got_corr;
  while (scan_it != scan_cloud->points.end ())
  {
    // reset variables for search
    min_dist2 = max_dist2;
    got_corr = false;
    // convert scan point to polar coords
    p_r = sqrt (scan_it->x * scan_it->x + scan_it->y * scan_it->y);
    p_theta = atan2 (scan_it->y, scan_it->x);
    // go over all model points
    model_it = model_cloud->points.begin ();
    while (model_it != model_cloud->points.end ())
    {
      // retrieve theta for the current model point
      m_theta = atan2 (model_it->y, model_it->x);
      // check if model point is in (angular) range
      if (fabs (angles::shortest_angular_distance (p_theta, m_theta)) < theta_threshold)
      {
        // get the range for the model point
        m_r = sqrt (model_it->x * model_it->x + model_it->y * model_it->y);
        // get sqrd dist between model point range and scan point range
        dist2 = (p_r - m_r) * (p_r - m_r);

        if (dist2 < min_dist2)
        {
          min_dist2 = dist2;
          best_it = model_it;
          got_corr = true;
        }
      }
      model_it++;
    }
    // if a correspondence was found, put it into the corr vec
    if (got_corr)
    {
      point_correspondences.push_back (CorrPair (*best_it, *scan_it));
    }
    scan_it++;
  }
}



void
calcICPCorrespondences (const sensor_msgs::PointCloud::ConstPtr &scan_cloud,
    const sensor_msgs::PointCloud::ConstPtr &model_cloud,
    CorrVec &point_correspondences)
{
  // clear old contents from correspondence vector
  point_correspondences.clear ();
  point_correspondences.reserve (scan_cloud->points.size ());
  float min_dist2, curr_dist2;
  bool got_corr;

  std::vector<geometry_msgs::Point32>::const_iterator scan_it, model_it,
    best_it;

  scan_it = scan_cloud->points.begin ();
  while (scan_it != scan_cloud->points.end ())
  {
    min_dist2 = max_dist2;
    got_corr = false;
    model_it = model_cloud->points.begin ();
    while (model_it != model_cloud->points.end ())
    {
      curr_dist2 = sqrdDist (*scan_it, *model_it);
      if (curr_dist2 < min_dist2)
      {
        min_dist2 = curr_dist2;
        best_it = model_it;
        got_corr = true;
      }
      model_it++;
    }
//    point_correspondences.push_back (CorrPair (*scan_it, *best_it));
    if (got_corr)
    {
      point_correspondences.push_back (CorrPair (*best_it, *scan_it));
    }
    scan_it++;
  }
}


tf::Transform
calcTransFormation (const CorrVec &point_correspondences)
{
  float c1_x = 0.0f, c1_y = 0.0f, c2_x = 0.0f, c2_y = 0.0f;
  float s_xx = 0.0f, s_xy = 0.0f, s_yx = 0.0f, s_yy = 0.0f;

  // compute centroids
  CorrVec::const_iterator it = point_correspondences.begin ();
  while (it != point_correspondences.end ())
  {
    c1_x += it->first.x;
    c1_y += it->first.y;

    c2_x += it->second.x;
    c2_y += it->second.y;
    it++;
  }

  c1_x /= point_correspondences.size ();
  c1_y /= point_correspondences.size ();
  c2_x /= point_correspondences.size ();
  c2_y /= point_correspondences.size ();
 
  // compute the other terms needed for error minimization (see slide 234)
  it = point_correspondences.begin ();
  while (it != point_correspondences.end ())
  {
    s_xx += (it->first.x - c1_x) * (it->second.x - c2_x);
    s_xy += (it->first.x - c1_x) * (it->second.y - c2_y);
    s_yx += (it->first.y - c1_y) * (it->second.x - c2_x);
    s_yy += (it->first.y - c1_y) * (it->second.y - c2_y);
    it++;
  }
  
  float theta = atan2 (s_yx - s_xy, s_xx + s_yy);
  float tx = c1_x - (c2_x * cos (theta) - c2_y * sin (theta));
  float ty = c1_y - (c2_x * sin (theta) + c2_y * cos (theta));

  // create and return a tf::Transform from the given information
  // As in previous homeworks
  tf::Transform trans;
  tf::Quaternion rot;
  rot.setEuler (0.0f, 0.0f, theta);

  trans.setRotation (rot);
  trans.setOrigin (tf::Vector3 (tx, ty, 0.0));
  return trans;
}

void
transformPointCloud (const sensor_msgs::PointCloud::Ptr &in_cloud,
    sensor_msgs::PointCloud::Ptr &out_cloud, const tf::Transform &transform)
{
  // clear and reserve memory for out_cloud
  out_cloud->points.clear ();
  out_cloud->points.reserve (in_cloud->points.size ());
  std::vector<geometry_msgs::Point32>::const_iterator p_it = in_cloud->points.begin ();
  geometry_msgs::Point32 p;
  tf::Vector3 v;
  // apply transformation to every point in in_cloud and add them to out_cloud
  while (p_it != in_cloud->points.end ())
  {
    v = tf::Vector3 (p_it->x, p_it->y, p_it->z);
    v = global_transform * v;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    out_cloud->points.push_back (p);
    p_it++;
  }
}

void
laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan,
    const sensor_msgs::LaserScan::ConstPtr& model)
{
  static ros::Time last = scan->header.stamp;

  if(scan->header.stamp < last) {
    global_transform.setIdentity ();
  }
  last = scan->header.stamp;

  static tf::TransformBroadcaster br;

  // since we later on work with translation in x and y it makes sense to
  // convert both scans to PointClouds first
  sensor_msgs::PointCloud::Ptr tmp_cloud (new sensor_msgs::PointCloud);
  sensor_msgs::PointCloud::Ptr scan_cloud (new sensor_msgs::PointCloud);
  sensor_msgs::PointCloud::Ptr model_cloud (new sensor_msgs::PointCloud);

  // convert model and scan to point clouds
  projector_.projectLaser (*model, *model_cloud);
  projector_.projectLaser (*scan, *tmp_cloud);

  // apply current pose estimate to scan cloud
  transformPointCloud (tmp_cloud, scan_cloud, global_transform);

  //just for visualization
  scan_cloud->header = model_cloud->header;
  pre_cloud_pub.publish (scan_cloud);

  CorrVec icp_corrs;
  CorrVec imr_corrs;
  tf::Transform trans_icp, trans_imr, trans;
  // compute correspondences and transformation correction
  if (use_icp || use_idc)
  {
    calcICPCorrespondences (scan_cloud, model_cloud, icp_corrs);
    trans_icp = calcTransFormation (icp_corrs);
  }
  if (use_imr || use_idc)
  {
    calcIMRCorrespondences (scan_cloud, model_cloud, imr_corrs);
    trans_imr = calcTransFormation (imr_corrs);
  }

  // compute next transformation step
  if (use_icp)
  {
    trans = trans_icp;
  }
  else if (use_imr)
  {
    trans = trans_imr;
  }
  else
  {
    trans.setOrigin (trans_icp.getOrigin ());
    trans.setRotation (trans_imr.getRotation ());
  }

  // update global pose with correction
  global_transform = global_transform * trans;

  // draw those cute lines
  if (use_icp || use_idc)
  {
    visualization_msgs::Marker icp_lines;
    icp_lines.header = model->header;
    icp_lines.ns = "icp_cors";
    icp_lines.id = 0;
    icp_lines.action = visualization_msgs::Marker::ADD;
    icp_lines.type = visualization_msgs::Marker::LINE_LIST;
    icp_lines.pose.orientation.w = 1.0;
    icp_lines.scale.x = 0.01;
    icp_lines.color.r = 0.0;
    icp_lines.color.g = 1.0;
    icp_lines.color.b = 1.0;
    icp_lines.color.a = 1.0;
    drawLines (icp_corrs, icp_lines);

    line_pub.publish (icp_lines);
  }
  
  if (use_imr || use_idc)
  {
    visualization_msgs::Marker imr_lines;
    imr_lines.header = model->header;
    imr_lines.ns = "imr_corrs";
    imr_lines.id = 0;
    imr_lines.action = visualization_msgs::Marker::ADD;
    imr_lines.type = visualization_msgs::Marker::LINE_LIST;
    imr_lines.pose.orientation.w = 1.0;
    imr_lines.scale.x = 0.01;
    imr_lines.color.r = 1.0;
    imr_lines.color.g = 0.0;
    imr_lines.color.b = 1.0;
    imr_lines.color.a = 1.0;
    drawLines (imr_corrs, imr_lines);

    line_pub.publish (imr_lines);
  }
  
  // just some output to console
  tf::Quaternion rot = global_transform.getRotation ();
  double roll, pitch, yaw;
  tf::Matrix3x3 (rot).getRPY (roll, pitch, yaw);
  tf::Vector3 translation = global_transform.getOrigin ();
  ROS_INFO ("published transformation %lf %lf %lf", translation[0], translation[1], yaw);
  
  // broadcast global current transformation between the two scans
  br.sendTransform (tf::StampedTransform (global_transform, model->header.stamp, model->header.frame_id, scan->header.frame_id));

  //just for visualization
  sensor_msgs::PointCloud::Ptr transformed_cloud (new sensor_msgs::PointCloud);
  transformed_cloud->header = model_cloud->header;
  transformPointCloud (tmp_cloud, transformed_cloud, global_transform);
  post_cloud_pub.publish (transformed_cloud);
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "icp_correspondences");
  ros::NodeHandle n;
  
  // quick and dirty way to determine algorithm
  use_icp = true; 
  use_imr = false;
  use_idc = false;
  if (argc > 1)
  {
    std::string idc = "--idc";
    std::string imr = "--imr";
    if (idc.compare (argv[1]) == 0)
    {
      use_icp = false;
      use_idc = true;
    }
    else if (imr.compare (argv[1]) == 0)
    {
      use_icp = false;
      use_imr = true;
    }
  }

  max_dist2 = 0.3f * 0.3f;  // no correspondence if points are farther apart than 0.3m
  theta_threshold = angles::from_degrees (5.0);

  // init transformation estimate
  global_transform.setIdentity ();

  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub (n, "/scan", 1);
  message_filters::Subscriber<sensor_msgs::LaserScan> model_sub (n, "/model", 1);

  message_filters::Synchronizer<MySyncPolicy> sync (MySyncPolicy (10), scan_sub, model_sub);
  sync.registerCallback (boost::bind (&laserCallback, _1, _2));
	
  line_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker", 10);
  post_cloud_pub = n.advertise<sensor_msgs::PointCloud> ("post_trans_cloud", 10);
  pre_cloud_pub = n.advertise<sensor_msgs::PointCloud> ("pre_trans_cloud", 10);

  ros::spin ();
  return 0;
}
