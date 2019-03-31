#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <iostream>
#include <cmath>

#define PI 3.14159265359

using namespace std;

double x, y, th, v, vth;
double goal_x, goal_y;
double linear_x, angular_z;

//Set linear velocity for turtle
double linearVel()
{
    linear_x = 0.5 * sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    if(linear_x < 0.001) linear_x = 0.0;
    return linear_x;
}

//Set angular velocity for turtle
double angularVel()
{
    angular_z = 2.0 * (atan2(goal_y - y, goal_x - x) - th);
    if(linear_x < 0.001) angular_z = 0.0;
    return angular_z;
}

//Update posision of turtle 
void poseUpdate(const turtlesim::Pose::ConstPtr& pose)
{
    x = pose->x, y = pose->y, th = pose->theta,
    v = pose->linear_velocity, vth = pose->angular_velocity;
    
    //adjust angular state of turtle in range [-PI, PI]
    if(th > PI) th -= 2 * PI;
    if(th < -PI) th += 2 * PI;
    
    //Output turtle's state on screen
    cout << x << " " << y << " " << th << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "haha");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/turtle1/pose", 100, &poseUpdate);
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

    geometry_msgs::Twist vel_msg;

    ros::Rate rate(100);
    while(node.ok())
    {
        //Check linear_velocity and angular_velocity. If they are 0.0, set new goal.
        if(linearVel() == 0 && angularVel() == 0)
        {
            cout << "Input goal x: "; cin >> goal_x;
            cout << "Input goal y: "; cin >> goal_y;
        }
        
        //Set linear_velocity and angular_velocity
        vel_msg.linear.x = linearVel();
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;

        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = angularVel();

        pub.publish(vel_msg);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
