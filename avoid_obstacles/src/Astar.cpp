#include "Astar.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

//TEMPORARY FOR PAST C array IMPLEMENTATION
#define NUM_ROWS 150
#define NUM_COLS 150
#define NUM_THETA 8
#define MAX_OPEN 10000

//delta_x for 6 motions, 8 orientations in 45 deg steps (0 to 315)
const int delta_x[6][8] = {{-1,-1,1,1,1,1,-1,-1},
					 {-1,-1,-1,1,1,1,1,-1},
					 {-1,-1,0,1,1,1,0,-1},
					 {1,1,0,-1,-1,-1,0,1},
					 {1,1,-1,-1,-1,-1,1,1},
					 {1,1,1,-1,-1,-1,-1,1}};
const int delta_y[6][8] = {{-1,-1,-1,-1,1,1,1,1},
					 {1,-1,-1,-1,-1,1,1,1},
					 {0,-1,-1,-1,0,1,1,1},
					 {0,1,1,1,0,-1,-1,-1},
					 {1,1,1,1,-1,-1,-1,-1},
					 {-1,1,1,1,1,-1,-1,-1}};
const int delta_theta[6] = {45,-45,0,0,45,-45};

/*int compareCells (const Astar::Cell& cellA, const Astar::Cell& cellB)
{
  return cellB.f > cellA.f? 1 : -1;
  //return cellB.f < cellA.f;
}*/
int compareCells (const void * a, const void * b)
{

  Astar::Cell *cellA = (Astar::Cell *)a;
  Astar::Cell *cellB = (Astar::Cell *)b;

  //This will not properly compare floats
  //return ( cellB->f - cellA->f );

  //The following ensures floats are compared properly
  // conditon ? IF_TRUE : IF_FALSE
  return cellB->f > cellA->f ? 1 : -1;
}

Astar::Astar():
		num_theta(8),
		map_x0(0.0),
		map_y0(0.0),
		map_res(0.5),
		obs_thresh(30)
{
	std::cout << "Astar initialized, num_theta: " << num_theta << "\n";
	std::cout << "Astar updated\n";
}

Astar::~Astar(){}

bool Astar::get_map_indices(float x, float y, int& ix, int& iy)
{
	ix = (x-map_x0)/map_res;
	iy = (y-map_y0)/map_res;
	return true;
}

int Astar::is_obs(nav_msgs::OccupancyGrid map, int ix, int iy)
{
	int ind = iy*NUM_COLS + ix;
	if(ind > map.data.size())
		return 1;
	if(map.data[iy*NUM_COLS + ix] > obs_thresh)
	{
		return 1;
	}
	return 0;
}
int Astar::is_obs2(nav_msgs::OccupancyGrid map, int ix, int iy)
{
	if(is_obs(map, ix, iy) == 1)
	{
		return 1;
	}
	else // check if two adjacent cells are obstacles
	{
		int count = 0;
		for(int dx = -1; dx <= 1; ++dx)
		{
			for(int dy = -1; dy <= 1; ++dy)
			{
				if(is_obs(map,ix+dx, iy+dy))
				{
					++count;
					if(count == 2)
						return 1;
				}
			}
		}
	}
	return 0;
}
bool Astar::get_path(geometry_msgs::Pose pose, geometry_msgs::Pose goal,
						nav_msgs::OccupancyGrid map, nav_msgs::Path& path)
{
	ros::Time start_time = ros::Time::now();
	ros::Duration MAX_PLAN_TIME(2.0);

	Cell open[MAX_OPEN] = {{0,0,0,0,0}};
	//std::vector<Cell> open;
	//open.resize(MAX_OPEN);
	/*open.push_back(new_cell(0,1,0,1,0));
	open.push_back(new_ckcell(1,2,0,2,0));
	open.push_back(new_cell(0.5,3,3,0,0));
	open.push_back(new_cell(2.0,4,4,0,0));
	open.push_back(new_cell(1.5,5,5,0,0));

	ROS_INFO("Get path test");
	printf("before sort\n");
	for(unsigned k=0; k<open.size(); ++k)
		printf("%f, %f, %f, %f, %d\n", open[k].f, open[k].g, open[k].x, open[k].y, open[k].theta);
	std::sort(open.begin(), open.end(), compareCells);
	printf("after sort\n");
	for(unsigned k=0; k<open.size(); ++k)
		printf("%f, %f, %f, %f, %d\n", open[k].f, open[k].g, open[k].x, open[k].y, open[k].theta);
	*/

	map_res = map.info.resolution;
	map_x0 = map.info.origin.position.x;
	map_y0 = map.info.origin.position.y;

	int finished[NUM_ROWS][NUM_COLS][NUM_THETA] = {{{0}}};
	int action[NUM_ROWS][NUM_COLS][NUM_THETA] = {{{0}}};
	//std::vector<int> finished;
	//std::vector<int> action;

	float x_init = pose.position.x;
	float y_init = pose.position.y;

	//get heading
	float yaw_deg = get_yaw(pose)*180.0/3.14159;

	float xg = goal.position.x;
	float yg = goal.position.y;
	float goal_deg = get_yaw(goal)*180.0/3.14159;
	int rg; // = boost::math::iround(-yg);
	int cg; // = boost::math::iround(xg);
	get_map_indices(xg, yg, cg, rg);
	int pg = boost::math::iround(goal_deg/45)%8;
	int thg = pg*45;

	float x1,y1,g1,f2,g2,h2,DIST,x2,y2,cost;
	int th1,th2,p1,p2;
	float next_pos[3] = {0,0,0};
	int r1,c1, nOpen, dCount, m, r2,c2, r_init, c_init, count;
	int numMotions = 6, rev_motion;
	int motions[6] = {-3,-2,-1, 1, 2, 3};
	int done, no_sol, a;
	x1 = x_init;
	y1 = y_init;
	//r1 = boost::math::iround(-y_init);
	//c1 = boost::math::iround(x_init);
	get_map_indices(x_init, y_init, c1, r1);
	p1 = boost::math::iround(yaw_deg/45)%8;
	th1 = p1*45;

	finished[r1][c1][p1] = 1;
	g1 = 0;
	//h2 = sqrt((xg-x1)^2 + (yg-y1)^2);
	//f2 = g1+h2;
	nOpen = 0;
	done = 0;
	no_sol = 0;
	DIST = map_res;

	printf("Begin while\n");
	dCount = 0;
	while(!done && !no_sol)
	{
		// calculate angle to goal
		float dx = xg-x1;
		float dy = yg-y1;
		int des_heading_deg = int(atan2(dy,dx)*180.0/3.1459+360)%360; //c % mod operator returns negative
		//md(5);
		for(m = 3; m<numMotions; m++)
		{
			cost = arc_move(next_pos,x1,y1,th1,motions[m],DIST);
			if(next_pos[0] == 32767)
			{
				printf("Invalid motion in move\n");
				return 1;
			}
			x2 = next_pos[0];
			y2 = next_pos[1];
			th2 = next_pos[2];
			//r2 = boost::math::iround(-y2);
			//c2 = boost::math::iround(x2);
			get_map_indices(x2, y2, c2, r2);
			p2 = (th2/45)%8;

			if(r2 >= 0 && r2 < NUM_ROWS && c2 >= 0 && c2 < NUM_COLS)
			{
				//if(map[r2][c2] == 0 && finished[r2][c2][p2] == 0)
				if(is_obs2(map,c2,r2) == 0 && finished[r2][c2][p2]==0)
				{
					//cost now taken care of in arc_move()

					//if(action[r1][c1][p1] * motions[m] < 0){cost = cost*10;} //Really slows it down, similar path in the end
					g2 = g1 + cost;// *DIST; ALREADY MULTIPLIED BY DIST IN arc_move

					//h2 = sqrt((xg-x2)*(xg-x2) + (yg-y2)*(yg-y2));
					h2 = (xg-x2)*(xg-x2) + (yg-y2)*(yg-y2);
					//ADD penalty to h2 based on turning toward goal or not
					h2 += fabs(float(th2-des_heading_deg)/100.0);
					f2 = g2*g2+h2;
					if(nOpen < MAX_OPEN)
					{
						nOpen += 1;
						//open.push_back(new_cell(0,0,0,0,0));
						open[nOpen-1].f = f2;
						open[nOpen-1].g = g2;
						open[nOpen-1].x = x2;
						open[nOpen-1].y = y2;
						open[nOpen-1].theta = th2;
					}
					else
					{
						printf("nOpen = %d\n",nOpen);
						open[0].f = f2;
						open[0].g = g2;
						open[0].x = x2;
						open[0].y = y2;
						open[0].theta = th2;
					}
					finished[r2][c2][p2] = 1;
					action[r2][c2][p2] = motions[m];
				}
			}
		}
		if(nOpen == 0)
		{
			no_sol = 1;
			printf("No Solution found\n");
		}
		else if( (ros::Time::now()-start_time)> MAX_PLAN_TIME )
		{
			no_sol = 1;
			printf("Timed Out, No Solution\n");
		}
		else
		{
			//md(6);
			if(nOpen > 1)
			{
				qsort(open,nOpen,sizeof(Cell),compareCells);
				//std::sort(open.begin(), open.end(), compareCells);
			}

			g1 = open[nOpen-1].g;
			x1 = open[nOpen-1].x;
			y1 = open[nOpen-1].y;
			th1 = open[nOpen-1].theta;
			dCount = dCount + 1;

			//r1 = boost::math::iround(-y1);
			//c1 = boost::math::iround(x1);
			get_map_indices(x1,y1,c1,r1);
			p1 = (th1/45)%8;
			nOpen -= 1;
			if(nOpen < 0)
			{
				nOpen = 0;
			}
			if(r1 == rg && c1 == cg) // && p1 == pg)
			{
				done = 1;
				printf("done\n");
			}
		}
	}

	if(done)
	{
		//md(7);

		//r_init = boost::math::iround(-y_init);
		//c_init = boost::math::iround(x_init);
		get_map_indices(x_init,y_init,c_init,r_init);

		//Step backwards and store the optimum path
		//printf("x: %.1f, y: %.1f\n",x1,y1);
		//fprintf(fpPath,"%.1f %.1f\n",x1,y1);
		path.poses.clear();
		geometry_msgs::PoseStamped wp;
		wp.pose.position.x = x1;
		wp.pose.position.y = y1;
		path.poses.push_back(wp);

		while( (abs(r1-r_init) > 0 || abs(c1-c_init) > 0) && dCount >= -2)
		{
			//printf("action: %d\n",action[r1][c1][p1]);
			rev_motion = -action[r1][c1][p1];
			cost = arc_move(next_pos,x1,y1,th1,rev_motion,DIST);
			if(next_pos[0] == 32767)
			{
				printf("Invalid motion in move\n");
				return 1;
			}
			x1 = next_pos[0];
			y1 = next_pos[1];
			th1 = next_pos[2];
			//r1 = boost::math::iround(-y1);
			//c1 = boost::math::iround(x1);
			get_map_indices(x1, y1, c1, r1);
			p1 = (th1/45)%8;
			//count = count+1;
			//x(count) = x1;
			//y(count) = y1;
			//printf("x: %.1f, y: %.1f\n",x1,y1);
			//fprintf(fpPath,"%.1f %.1f\n",x1,y1);
			wp.pose.position.x = x1;
			wp.pose.position.y = y1;
			path.poses.push_back(wp);
			dCount = dCount - 1;
		}
		std::reverse(path.poses.begin(),path.poses.end());
		//plot(x,y,'g')
	}

	/*
	path.poses.clear();
	geometry_msgs::PoseStamped wp;
	wp.pose.position.x = 5.0;
	wp.pose.position.y = 5.0;
	path.poses.push_back(wp);
	wp.pose.position.x = 15.0;
	path.poses.push_back(wp);
	*/

	return true;
}

Astar::Cell Astar::new_cell(float f, float g, float x, float y, int theta)
{
	Cell c = {f,g,x,y,theta};
	return c;
}

float Astar::get_yaw(geometry_msgs::Pose pose)
{
	double roll, pitch, yaw;
	tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	tf::Matrix3x3 quat_matrix(q);
	quat_matrix.getRPY(roll, pitch, yaw);
	return (float)yaw;
}

float Astar::arc_move(float next_pos[], float x1, float y1, int th1, int motion, float d)
{
	//motion will be one of: -3(rev right),-2(rev left),-1(rev), 1(fwd), 2(fwd left), 3(fwd right)
	float cost;
	int m_index,p;
	int th2;

	m_index = motion+3 - (motion > 0);
	if(th1 < 0){th1 += 360;}
	if(th1 >= 360){th1 -= 360;}
	p = (th1/45)%8; //current theta index (0 to 7)

	float net_delta_x = float(delta_x[m_index][p])*d;
	float net_delta_y = float(delta_y[m_index][p])*d;
	next_pos[0] = x1 + net_delta_x; //x2
	next_pos[1] = y1 + net_delta_y; //y2
	th2 = th1 + delta_theta[m_index]; //th2
	if(th2 < 0){th2 += 360;}
	if(th2 >= 360){th2 -= 360;}
	next_pos[2] = th2;

	//cost = (abs(delta_x[m_index][p]) + abs(delta_y[m_index][p]));
	cost = (abs(delta_x[m_index][p]) + abs(delta_y[m_index][p])) > 1 ? 1.42*d : d;
	/*if(motion < 0)
	{
		cost = cost*20;
	}
	*/
	/*if(abs(motion) > 1)
	{
		cost = cost*1.4;
	}*/
	return cost;
}