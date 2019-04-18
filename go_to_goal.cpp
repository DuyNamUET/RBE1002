#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iostream>
#include <cmath>

#define PI 3.1415926536

using namespace std;

double x, y, th, v, vth;
double goal_x, goal_y;
double linear_x, angular_z;

//Set linear velocity for turtle
double linearVel()
{
    linear_x = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    if(linear_x < 0.001) linear_x = 0.0;
    return 1.0 * linear_x;
}

//Set angular velocity for turtle
double angularVel()
{
    angular_z = asin((cos(th) * (goal_y - y) - sin(th) * (goal_x - x)) / linear_x);
    if(linear_x < 0.001) angular_z = 0.0;
    return 4.0 * angular_z;
}

//Update posision of turtle 
void poseUpdate(const turtlesim::Pose::ConstPtr& pose)
{
    x = pose->x, y = pose->y, th = pose->theta,
    v = pose->linear_velocity, vth = pose->angular_velocity;
    if(th > PI) th -= 2 * PI;
    if(th < -PI) th += 2 * PI;
    cout << x << " " << y << " " << th << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "haha");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/turtle1/pose", 100, &poseUpdate);
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

    geometry_msgs::Twist vel_msg;

    int i = 1;

    ros::Rate rate(100);
    while(node.ok())
    {

        if(linearVel() == 0 && angularVel() == 0)
        {
            if(i < argc - 1)
            {
                goal_x = atof(argv[i++]);
                goal_y = atof(argv[i++]);
            }
        }
        vel_msg.linear.x = linearVel();
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;

        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = angularVel();

        pub.publish(vel_msg);
            
        if(i == argc) pub.publish(vel_msg);
        else if(linearVel() == 0 && angularVel() == 0) break;

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

/*
Run demo
$ rosrun [package] go_to_goal 1 2 6 5 0 9
*/
