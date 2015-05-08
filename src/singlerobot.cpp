#include "ros/ros.h"
#include <math.h>
#include <string>
#include <sstream>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "singlerobot/message.h"
#include "singlerobot/pose.h"
#include "rosgraph_msgs/Clock.h"
#include <string>
#include "pathplanner.h"

//using namespace std;
// This file will subscribe to IR data, Obstacle Sensor data and encoder data from Simulator and then plan the path and send the global data + bot data to coverage followed by receiving data from coverage.
//N Instances of this file will be run for each robot


// the arena on the simulator stretches from (-X_MAX/2, -Y_MAX/2) to (X_MAX/2, Y_MAX/2) 
const int X_MAX = 20;
const int Y_MAX = 20;
const int SCALE = 1; // Each cell of map[][] covers 1/SCALE on simulator
// hence higher the scale, more refined the path generated.
const double TOL = 1 / double(SCALE);


class swarmRobot
{
public:
	int id;
	swarmRobot()
	{
		n.getParam("ID",id);
         //id = 2;
    	n.getParam("NAME",botName);
		odomTopic << "/swarmbot" << id << "/odom";
		velTopic << "/swarmbot" << id << "/cmd_vel";
		messageTopic << "/swarmbot" << id << "/message";
		odomSub = n.subscribe<nav_msgs::Odometry>(odomTopic.str(),1000,&swarmRobot::robotCallback,this);
		velPub = n.advertise<geometry_msgs::Twist>(velTopic.str(), 50);
		clockSub = n.subscribe<rosgraph_msgs::Clock>("/clock", 1000, &swarmRobot::timeCallback, this);
		messagePub = n.advertise<singlerobot::message>(messageTopic.str(), 10);
		coverageSub = n.subscribe("/coverage", 50, &swarmRobot::coverageCallback, this);
		obstacleSub = n.subscribe("/obstacleList", 50, &swarmRobot::obstacleCallback, this);

		map = (int **) malloc(X_MAX * SCALE * sizeof(int *));

		for(int i = 0; i < X_MAX * SCALE; i++) {
			map[i] = (int *) malloc(Y_MAX * SCALE * sizeof(int));
		}

        // for(int i = 0; i < X_MAX * SCALE; ++i) {
        //     for(int j = 0; j < Y_MAX * SCALE; ++j) {
        //         map[i][j] = 0;
        //         //std::cout << "i = " << i << " j = " << j << std::endl;
        //     }
        // }

		obstacles_updated = false;
		obstacles_set = false;
		pos_set = false;
		destination_set = false;
	}
    ~swarmRobot() {
        for(int i = 0; i < X_MAX * SCALE; i++) {
            free(map[i]);
        }
        free(map);
    }
	void robotCallback(nav_msgs::Odometry msg)
	{
        // std::cout << "robotCallback called" << std::endl;
		current_pos = msg.pose.pose;

		pos_set = true;

        // std::cout << "robotCallback exited" << std::endl;
	}
	void timeCallback(rosgraph_msgs::Clock msg)
	{
		clockTime = msg;
	}
	void coverageCallback(const swarm_simulator::obstacleList msg) {
        // std::cout << "coverageCallback called" << std::endl;
		for(int i = 0; i < msg.obstacles.size(); ++i){
			if(msg.obstacles[i].shape == id) {
				destination_pos.position.x = msg.obstacles[i].x;
				destination_pos.position.y = msg.obstacles[i].y;
			}
		}
		destination_set = true;

        // std::cout << "coverageCallback exited" << std::endl;
	}
	void obstacleCallback(const swarm_simulator::obstacleList msg) {

        // std::cout << "obstacleCallback called" << std::endl;
		obstacles_updated = true;
		obstacles_set = true;

		std::vector<swarm_simulator::obstacleData> obstacles;
		obstacles.clear();
		for(int i = 0; i < msg.obstacles.size(); ++i){
			if((msg.obstacles[i].radius > 1.4 && msg.obstacles[i].radius < 1.6) && msg.obstacles[i].shape == id) {
			}
			else {
				obstacles.push_back(msg.obstacles[i]);
			}
		}

        // std::cout << "obstacles.size() = " << obstacles.size() << std::endl;

        // std::cout << "map = " << map << std::endl;
        // std::cout << "map[0] = " << map[0] << std::endl;
        // std::cout << "map[0][0] = " << map[0][0] << std::endl;

		fillMap(map, X_MAX, Y_MAX, SCALE, obstacles);
        // std::cout << "fillmap exited" << std::endl;

        // std::cout << "obstacleCallback exited" << std::endl;
	}
	void messageFiller(){
		msg.id = id;
		msg.stamp = clockTime.clock;
		singlerobot::pose temp;
		temp.x = current_pos.position.x;
		temp.y = current_pos.position.y;
		temp.theta = theta(current_pos);
		temp.id = id;
		msg.table.push_back(temp);
	}
	void printPositions(void)
	{
		printf("%lf %lf %lf\n",current_pos.position.x, current_pos.position.y, current_pos.position.z);
	}
	void run()
	{
        ros::Rate loop_rate(50);

        while(ros::ok())
        {
            ros::spinOnce();
            if(pos_set)
                break;
            loop_rate.sleep();
        }

        Pose waypoint = current_pos;
        std::vector<Point> path;        
        Twist cmd_vel;

        while(ros::ok()) {
            messageFiller();
            messagePub.publish(msg);
            std::cout << "Publishing" << std::endl;


            while(!obstacles_set || !pos_set  || !destination_set) {
                std::cout << "Waiting for obstacle list" <<  std::endl;
                ros::spinOnce();
                loop_rate.sleep();
            }

            std::cout << "Reached here" << std::endl;

            if(obstacles_updated) { //find new path

                long destination_x = std::lround((destination_pos.position.x + X_MAX / 2) * SCALE);
                long destination_y = std::lround((destination_pos.position.y + Y_MAX / 2) * SCALE);
                if(destination_x < 0) destination_x = 0;
                if(destination_y < 0) destination_y = 0;
                if(destination_x >= X_MAX * SCALE) destination_x = X_MAX * SCALE - 1;
                if(destination_y >= Y_MAX * SCALE) destination_y = Y_MAX * SCALE - 1;

                long current_x = std::lround((current_pos.position.x + X_MAX / 2) * SCALE);
                long current_y = std::lround((current_pos.position.y + Y_MAX / 2) * SCALE);
                if(current_x < 0) current_x = 0;
                if(current_y < 0) current_y = 0;
                if(current_x >= X_MAX * SCALE) current_x = X_MAX * SCALE - 1;
                if(current_y >= Y_MAX * SCALE) current_y = Y_MAX * SCALE - 1;

                path = AStar(map, X_MAX, Y_MAX, SCALE, Point(current_x, current_y), Point(destination_x, destination_y));

    	       //waypoint = current_pos;

                printMap(map, X_MAX, Y_MAX, SCALE);

                obstacles_updated = false;
            }

            //reached waypoint
                if (!path.empty()) { //next waypoint available
			if(current_pos.position.x > waypoint.position.x - TOL &&
               current_pos.position.x < waypoint.position.x + TOL &&
               current_pos.position.y > waypoint.position.y - TOL &&
    	       current_pos.position.y < waypoint.position.y + TOL) { 
                    path.pop_back();
                    Point target = path.back();
                    waypoint.position.x = (target.first / SCALE) - (X_MAX / 2);
                    waypoint.position.y = (target.second / SCALE) - (Y_MAX / 2);

                    // if(!path.empty()) { //might make things smoother
                    //   Point next = path.back();
                    //   double angle = normalizeAngle(atan2(next.second - target.second, next.first - target.first));
                    //   waypoint.orientation.z = sin(angle/2);
                    //   waypoint.orientation.w = cos(angle/2);
                    // }
                }
                }
                else //reached destination
                {
                    std::cout << "Path empty" << std::endl;
                    cmd_vel.angular.z = 0;
                    cmd_vel.linear.x = 0;
                    velPub.publish(cmd_vel);
                    break;
                }
            
            calculate_u_omega(current_pos, waypoint, cmd_vel);
            velPub.publish(cmd_vel);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }


protected:
	singlerobot::message msg;
	ros::Subscriber odomSub;
	ros::Subscriber clockSub;
	ros::Publisher messagePub;
	rosgraph_msgs::Clock clockTime;
	ros::NodeHandle n;
	std::string botName;
	//double posX,posY,posZ;
	std::stringstream messageTopic, odomTopic;

	int **map;
	std::stringstream velTopic;
	ros::Publisher velPub;
	ros::Subscriber coverageSub, obstacleSub;

	Pose current_pos;
	Pose destination_pos;
	bool obstacles_updated;
	bool obstacles_set;
	bool pos_set;
	bool destination_set;
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "singlerobot");
	swarmRobot bot;	

	bot.run();

	return 0;
}
