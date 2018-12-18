#include <ros/ros.h>

#include <angles/angles.h>
#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/LU>

ros::Duration save_interval (10.0);
ros::Time last_interval_stamp (0.0);
float cell_size = .02f;

ros::Publisher vis_pub;

/*
   Function to compute the map coordinates for a given point in
   cartesian coordinates
   @param[in] coords An Eigen::Vector2f containing cartesian coordinates
   @param[out] min_point An Eigen::Vector2f denoting the cartesian coordinates
   of the 'lower left' map corner
   @param[in] cell_size the side length of a single map cell (assuming squared
   cells)
   @return The map coordinates as an Eigen::Vector2i
 */
Eigen::Vector2i
cartesianToMap (const Eigen::Vector2f &coords,
    const Eigen::Vector2f &min_point, float cell_size)
{
  Eigen::Vector2f tmp;
  tmp = coords - min_point;
  tmp /= cell_size;
  Eigen::Vector2i map_coords (static_cast<int> (round (tmp(0))),
      static_cast<int> (round (tmp(1))));
  return map_coords;
}

void
saveMapToPGM(const std::string &filename, unsigned char **pixels,
    unsigned int x_dim, unsigned int y_dim)
{
  // open file and write file
  std::ofstream file (filename.c_str ());
  // change x and y dimension (since x is 'forward' / 'up')
  file << "P2\n" << y_dim << " " << x_dim << "\n255" << std::endl;

  // write image from 'bottom' to 'top' and 'right' to 'left' so
  // that it better corresponds to the ros coordinate system
  for (size_t i = 0; i < x_dim; ++i)
  {
    for (size_t j = 0; j < y_dim; ++j)
    {
      file <<  static_cast<unsigned int> (pixels[(x_dim - 1) - i][(y_dim - 1) - j]) << " ";
    }
    file << std::endl;
  }
  // flush and close the stream
  file.flush ();
  file.close ();
}


/* Method for drawing a Bresenham line using the code straight from
   wikipedia.
   @param[in] map The map / image onto which the line should be drawn
   @param[in] line_origin The start point of the line as an Eigen::Vector2i
   @param[in] line_end The end point of the line as an Eigen::Vector2i
   @param[in] line_val The value which is written into the image for coordinates belonging to the line
   */
void
drawBresenhamLine (unsigned char **map, const Eigen::Vector2i &line_origin,
    const Eigen::Vector2i &line_end, unsigned char line_val)
{
  int dx, dy, x, y, sx, sy, error, e2;
  dx = abs (line_end (0) - line_origin (0));
  dy = abs (line_end (1) - line_origin (1));
  line_origin (0) < line_end (0)? sx = 1 : sx = -1;
  line_origin (1) < line_end (1)? sy = 1 : sy = -1;
  error = dx - dy;
  x = line_origin (0);
  y = line_origin (1);

  while (!(x == line_end (0) && y == line_end (1)))
  {
    if (x < 0 || y < 0)
    {
      ROS_ERROR ("Either x or y or both are smaller than 0. Skipping map entry.");
      continue;
    }
    map[x][y] = line_val;
    e2 = 2 * error;
    if (e2 > -dy)
    {
      error = error - dy;
      x += sx;
    }
    if (e2 < dx)
    {
      error = error + dx;
      y += sy;
    }
  }
}

/* Function to retrieve the dimensions (i.e. side length of axis-aligned
   bounding box) for a given point cloud.
   @param[in] cloud The point cloud for which the dimensions should be determined
   @param[out] x_dim The dimension of the bounding box along the x-direction
   @param[out] y_dim The dimension of the bounding box along the y-direction
   @return The cartesian coordinates of the lower corner of the bounding box (i.e. minimal x- and y- values)
   */
Eigen::Vector2f
getCloudDimensions (const sensor_msgs::PointCloud &cloud,
    float &x_dim, float &y_dim)
{
  // initalize min and max points
  // make sure that the origin(0/0) (i.e. robots position) is part of the map
  Eigen::Vector2f min(0.0, 0.0);
  Eigen::Vector2f max(0.0, 0.0);

  // retrieve corners of axis-aligned boundingbox for point cloud
  std::vector<geometry_msgs::Point32>::const_iterator p_it;
  p_it = cloud.points.begin ();
  while (p_it != cloud.points.end ())
  {
	 max(0) = std::max(max(0), p_it->x);
	 min(0) = std::min(min(0), p_it->x);

	 max(1) = std::max(max(1), p_it->y);
	 min(1) = std::min(min(1), p_it->y);

    p_it++;
  }

  // compute dimensions
  x_dim = max(0) - min(0);
  y_dim = max(1) - min(1);

  return min;
}

void laserCallback(sensor_msgs::LaserScan scan)
{
  // sanitize scan
  for(size_t i = 0; i < scan.ranges.size(); ++i)
  {
    if(isnan(scan.ranges[i]) || scan.ranges[i] > scan.range_max)
      scan.ranges[i]= scan.range_max;
  }

  // create a new point cloud
  sensor_msgs::PointCloud cloud;

  laser_geometry::LaserProjection projector;
  projector.projectLaser(scan, cloud, scan.range_max+1);

  // retrieve the dimensions of the current scan
  float x_dim, y_dim;
  Eigen::Vector2f min_point;
  min_point = getCloudDimensions (cloud, x_dim, y_dim);

  // determine how many cells we need
  unsigned int nr_x_cells, nr_y_cells;
  nr_x_cells = static_cast<unsigned int>(ceil (x_dim / cell_size)) + 1;
  nr_y_cells = static_cast<unsigned int>(ceil (y_dim / cell_size)) + 1;

  // allocate the map
  unsigned char **map = new unsigned char*[nr_x_cells];
  for (size_t i = 0; i < nr_x_cells; ++i)
  {
    map[i] = new unsigned char[nr_y_cells];
  }

  // initialize all cells as 'unknown'
  for (size_t i = 0; i < nr_x_cells; ++i)
  {
    for (size_t j = 0; j < nr_y_cells; ++j)
    {
      map[i][j] = static_cast<unsigned char>(128);
    }
  }

  // retrieve the map coordinated of the origin (scanner position)
  Eigen::Vector2f origin (0.0, 0.0);
  Eigen::Vector2i origin_mc = cartesianToMap (origin, min_point, cell_size);
  if (origin_mc(0) < 0 || origin_mc(1) < 0 ||
      origin_mc(0) >= static_cast<int> (nr_x_cells) ||
      origin_mc(1) >= static_cast<int> (nr_y_cells))
  {
    ROS_ERROR ("origin (%d, %d) out of bounds", origin_mc(0),
        origin_mc(1));
    ROS_ERROR ("min_point: (%f, %f)", min_point (0), min_point (1));
  }
  Eigen::Vector2f coords;
  Eigen::Vector2i map_coords;

  // loop over all points in the point cloud
  std::vector<geometry_msgs::Point32>::const_iterator p_it;
  p_it = cloud.points.begin ();

  while (p_it != cloud.points.end ())
  {
    // retrieve the map coordinates for the scan point
    coords(0) = p_it->x;
    coords(1) = p_it->y;
    map_coords = cartesianToMap (coords, min_point, cell_size);

    if (map_coords(0) < 0 || map_coords(1) < 0 ||
        map_coords(0) >= static_cast<int> (nr_x_cells) ||
        map_coords(1) >= static_cast<int> (nr_y_cells))
    {
      ROS_ERROR ("map coords (%d, %d) out of bounds", map_coords(0),
          map_coords(1));
    }
    // store scan point in map as 'occupied'
    map[map_coords(0)][map_coords(1)] = 0;

    // use bresenham line from origin to scan point to mark 'unoccupied' cells
    drawBresenhamLine (map, origin_mc, map_coords, 255);
    p_it++;
  }

  // check if it's time to save the map to file
  ros::Duration d = cloud.header.stamp - last_interval_stamp;
  if (d > save_interval)
  {
    static unsigned int file_counter = 0;
    // write map to file
    std::stringstream ss;
    ss << "map_" << file_counter++ << ".pgm";
    saveMapToPGM (ss.str (), map, nr_x_cells, nr_y_cells);
    ROS_INFO ("wrote map to file '%s'", ss.str ().c_str ());
    last_interval_stamp = cloud.header.stamp;
  }

  // publish map to ROS for rviz visualization
  nav_msgs::OccupancyGrid grid;
  grid.header = cloud.header;
  grid.data.resize (nr_x_cells * nr_y_cells);
  grid.info.width = nr_x_cells;
  grid.info.height = nr_y_cells;
  grid.info.resolution = cell_size;
  grid.info.origin.position.x = min_point (0);
  grid.info.origin.position.y = min_point (1);
  size_t index = 0;
  for (size_t i = 0; i < nr_y_cells; ++i)
  {
    for (size_t j = 0; j < nr_x_cells; ++j)
    {
      if (map[j][i] == 0)
      {
        grid.data[index++] = 100;
      }
      else if (map[j][i] == 255)
      {
        grid.data[index++] = 0;
      }
      else
      {
        grid.data[index++] = -1;
      }
    }
  }
  vis_pub.publish (grid);

  // free allocated memory
  for (size_t i = 0; i < nr_x_cells; ++i)
  {
    delete[] map[i];
  }
  delete[] map;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "grid_mapper");

  ros::NodeHandle n;

  vis_pub = n.advertise<nav_msgs::OccupancyGrid> ("grid_map", 0 );
  ros::Subscriber sub = n.subscribe("/scan", 1, laserCallback);

  ros::spin();

  return 0;
}
