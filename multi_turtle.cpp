/*
Describe:
    - We have 3 turtle with random location.
    - Unknown number of checkpoints and each location of checkpoint is input from keyboard.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <iostream>
#include <cmath>

#define PI 3.1415926536

using namespace std;

turtlesim::Pose pose1, pose2, pose3;

//Update pose for each turtle
void updatePose1(const turtlesim::PoseConstPtr &pose)
{
    pose1 = *pose;
}

void updatePose2(const turtlesim::PoseConstPtr &pose)
{
    pose2 = *pose;
}

void updatePose3(const turtlesim::PoseConstPtr &pose)
{
    pose3 = *pose;
}

//Set distance form turtle to checkpoint
double setDistance(turtlesim::Pose pose, double goal_x, double goal_y)
{
    double dis = sqrt(pow(goal_x - pose.x, 2) + pow(goal_y - pose.y, 2));
    if(dis < 0.01) dis = 0.0;
    return dis;
}

//Set angurlar form turtle to checkpoint
double setAngular(turtlesim::Pose pose, double goal_x, double goal_y)
{
    double ang;
    if(setDistance(pose, goal_x, goal_y) < 0.01) ang = 0.0;
    else
    {
        ang = asin((cos(pose.theta) * (goal_y - pose.y) - sin(pose.theta) * (goal_x - pose.x))
                    / setDistance(pose, goal_x, goal_y));
    }
    return ang;
}

//Get velocity
geometry_msgs::Twist getVelocity(turtlesim::Pose pose, double goal_x, double goal_y)
{
    geometry_msgs::Twist vel;

    vel.linear.x = 1.5 * setDistance(pose, goal_x, goal_y);
    vel.angular.z = 10.0 * setAngular(pose, goal_x, goal_y);
    return vel;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_turtle");
    ros::NodeHandle node;

    //Declare random pose of multiple turtles
    for(int i = 1; i < 3; i++)
    {
        ros::service::waitForService("spawn");
        ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn turtle;
        turtle.request.x = rand() % 12;
        turtle.request.y = rand() % 12;
        turtle.request.theta = 0;
        spawner.call(turtle);
    }

    ros::Subscriber sub1 = node.subscribe("/turtle1/pose", 100, updatePose1);
    ros::Subscriber sub2 = node.subscribe("/turtle2/pose", 100, updatePose2);
    ros::Subscriber sub3 = node.subscribe("/turtle3/pose", 100, updatePose3);

    ros::Publisher pub1 = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    ros::Publisher pub2 = node.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 100);
    ros::Publisher pub3 = node.advertise<geometry_msgs::Twist>("/turtle3/cmd_vel", 100);

    //Put pose's turtle to array, each pose is one column 2D
    double arr_goal[3][2];
    arr_goal[0][0] = pose1.x;
    arr_goal[0][1] = pose1.y;
    int i = 1;

    ros::Rate rate(100);
    while(node.ok())
    {
        //Did each turtle go to checkpoint. If done, go to new checkpoint
        if(setDistance(pose1, arr_goal[0][0], arr_goal[0][1]) == 0.0)
        {
            if(i < argc - 1)
            {
                arr_goal[0][0] = atof(argv[i++]);
                arr_goal[0][1] = atof(argv[i++]);
                cout << "turtle1: " << arr_goal[0][0] << " " << arr_goal[0][1] << endl;
            }
        }

        if(setDistance(pose2, arr_goal[1][0], arr_goal[1][1]) == 0.0)
        {
            if(i < argc - 1)
            {
                arr_goal[1][0] = atof(argv[i++]);
                arr_goal[1][1] = atof(argv[i++]);
                cout << "turtle2: " << arr_goal[1][0] << " " << arr_goal[1][1] << endl;
            }
        }

        if(setDistance(pose3, arr_goal[2][0], arr_goal[2][1]) == 0.0)
        {
            if(i < argc - 1)
            {
                arr_goal[2][0] = atof(argv[i++]);
                arr_goal[2][1] = atof(argv[i++]);
                cout << "turtle3: " << arr_goal[2][0] << " " << arr_goal[2][1] << endl;
            }
        }

        pub1.publish(getVelocity(pose1, arr_goal[0][0], arr_goal[0][1]));
        pub2.publish(getVelocity(pose2, arr_goal[1][0], arr_goal[1][1]));
        pub3.publish(getVelocity(pose3, arr_goal[2][0], arr_goal[2][1]));

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
