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

// This file will subscribe to IR data, Obstacle Sensor data and encoder data from Simulator and then plan the path and send the global data + bot data to coverage followed by receiving data from coverage.
//N Instances of this file will be run for each robot

class swarmRobot
{
  public:
    std::stringstream messageTopic;
    int id;
    swarmRobot(): posX(0.0),posY(0.0),posZ(0.0)
    {
        odomSub = n.subscribe<nav_msgs::Odometry>("/swarmbot1/odom",1000,&swarmRobot::robotCallback,this);
        clockSub = n.subscribe<rosgraph_msgs::Clock>("/clock", 1000, &swarmRobot::timeCallback, this);
        messagePub = n.advertise<singlerobot::message>(messageTopic.str(), 1);
    }
    void robotCallback(nav_msgs::Odometry msg)
    {
	     posX = msg.pose.pose.position.x;
	     posY= msg.pose.pose.position.y;
	     posZ = msg.pose.pose.position.z;
    }
    void timeCallback(rosgraph_msgs::Clock msg)
    {
       clockTime = msg;
    }
    void messageFiller(){
       msg.id = id;
       msg.stamp = clockTime.clock;
       singlerobot::pose temp;
       temp.x = posX;
       temp.y = posY;
       temp.theta = posZ;
       temp.id = id;
       msg.table.push_back(temp);
    }
    void printPositions(void)
    {
	     printf("%lf %lf %lf\n",posX,posY,posZ);
    }
    void run()
    {
       messageFiller();
       messagePub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();

	     // printPositions();
    }	
    ros::Rate loop_rate;

  protected:
    singlerobot::message msg;
    ros::Subscriber odomSub;
    ros::Subscriber clockSub;
    ros::Publisher messagePub;
    rosgraph_msgs::Clock clockTime;
    ros::NodeHandle n;
    double posX,posY,posZ;



};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "singlerobot");
      
  swarmRobot bot;
  bot.messageTopic << "/swarmbot";
  ros::Rate lp(5);
  bot.loop_rate = lp;
  bot.id = (int)(argv[1][0]-'0');
  std::cout << "BOT ID " << bot.id << std::endl;
  bot.messageTopic << bot.id << "/message";
  while(ros::ok())
      bot.run();
  
  return 0;
}
