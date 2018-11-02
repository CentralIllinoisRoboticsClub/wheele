#ifndef Astar_H
#define Astar_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <vector>

class Astar
{
public:
	Astar();
	~Astar();

	bool get_path(geometry_msgs::Pose pose, geometry_msgs::Pose goal,
					nav_msgs::OccupancyGrid map, nav_msgs::Path& path);

	//The a-star path-finding data of a map grid cell
	typedef struct {
		float f; //f = g+heuristic
		float g; //cumulative motion cost, should be actual distance, maybe weight reverse
		float x; //x coordinate in meters
		float y; //y coordinate in meters
		int theta; //orientation in deg (45 deg res)
	}Cell;

private:
	float arc_move(float next_pos[], float x1, float y1, int th1, int motion, float d);

	//bool compareCells(const Cell& a, const Cell& b);

	Cell new_cell(float f, float g, float x, float y, int theta);

	float get_yaw(geometry_msgs::Pose pose);

	bool get_map_indices(float x, float y, int& ix, int& iy);

	int is_obs(nav_msgs::OccupancyGrid map, int ix, int iy);

	int num_theta;

	float map_x0, map_y0, map_res;

	int obs_thresh;

};

#endif
