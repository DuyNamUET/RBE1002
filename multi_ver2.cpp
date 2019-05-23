/*********************************************************************
** run demo: rosrun [package] [type] [list]                         **
** [list] is array of number (argv(1) to argv(end))                 **
** argv(1) is number of turtles that we have                        **
** from argv(2) to argv(2 * argv(1) + 1) is position of each turtle **
** from argv(2 * (argv(1) + 1)) is position of each checkpoint      **
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>

class TTurtle
{
    public:
    int index;
    ros::Subscriber sub;
    ros::Publisher pub;
    turtlesim::Pose pose;

    void poseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        pose = *msg;
    }
};

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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hoho");
	ros::NodeHandle node;

	int num = atoi(argv[1]);
    TTurtle tturtle[num];

    for(int i = 0; i < num; i++)
    {
        std::stringstream s;
        s << "turtle" << i + 1;
        std::string name = s.str();

        if(i != 0)
        {
            ros::service::waitForService("spawn");
            ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
            turtlesim::Spawn turtle;
            turtle.request.x = atof(argv[2 * i]);
            turtle.request.y = atof(argv[2 * i + 1]);
            spawner.call(turtle);
        }

        tturtle[i].index = i;
        tturtle[i].sub = node.subscribe(name + "/pose", 100, &TTurtle::poseCallback, &tturtle[i]);
        tturtle[i].pub = node.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 100);
    }

    //Put pose's turtle to array, each pose is one column 2D
    double arr_goal[num][2];
    for(int i = 0; i < num; i++)
    {
    	arr_goal[i][0] = tturtle[i].pose.x;
    	arr_goal[i][1] = tturtle[i].pose.y;
    }

    int id = 2 * num;

	ros::Rate rate(100);
	while(node.ok())
	{
		for(int i = 0; i < num; i++)
		{
			if(setDistance(tturtle[i].pose, arr_goal[i][0], arr_goal[i][1]) < 0.01)
	        {
	            if(id < argc - 1)
	            {
	                arr_goal[i][0] = atof(argv[id++]);
	                arr_goal[i][1] = atof(argv[id++]);
	                // cout << "turtle1: " << arr_goal[0][0] << " " << arr_goal[0][1] << endl;
	                ROS_INFO("turtle%d: %0.1f %0.1f", i + 1, arr_goal[i][0], arr_goal[i][1]);
	            }
	        }

	        tturtle[i].pub.publish(getVelocity(tturtle[i].pose, arr_goal[i][0], arr_goal[i][1]));
		}

		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
