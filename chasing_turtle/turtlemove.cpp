#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "cmath"

const double pid=0.8;

class turtlemove
{
    ros::NodeHandle nh;
    ros::Publisher velPublisher;
    ros::Subscriber poseSubscriber;
    ros::Subscriber goalSubscriber;
    turtlesim::Pose pose;
    turtlesim::Pose goal;
    int X_velocity;
    void poseCall(const turtlesim::Pose& newPose);
    void goalCall(const turtlesim::Pose& newGoal);
    public:turtlemove();
};

turtlemove::turtlemove()
{
    velPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel",500);
    poseSubscriber = nh.subscribe("pose",500,&turtlemove::poseCall,this);
    goalSubscriber = nh.subscribe("goal",500,&turtlemove::goalCall,this);
    ros::param::get("/X_velocity",X_velocity);
    goal.x=-1;
    goal.y=-1;
    goal.theta=0;
}

void turtlemove::goalCall(const turtlesim::Pose& newGoal)
{
    goal=newGoal;
}

void turtlemove::poseCall(const turtlesim::Pose& newPose)
{
    pose=newPose;
    if(goal.x>0){
        double goalAngel=atan2(goal.y-pose.y,goal.x-pose.x);
        double angleError=goalAngel-pose.theta;
        angleError=atan2(sin(angleError),cos(angleError));
        double Z_velocity=pid*angleError;
        if(abs(goal.x-pose.x)>0.5 || abs(goal.y-pose.y)>0.5){
        geometry_msgs::Twist vel;
        vel.linear.x = X_velocity;
        vel.linear.y=0;
        vel.linear.z=0;
        vel.angular.x=X_velocity;
        vel.angular.y=0;
        vel.angular.z=Z_velocity;
        } else{
        geometry_msgs::Twist vel;
        vel.linear.x = 0;
        vel.linear.y=0;
        vel.linear.z=0;
        vel.angular.x=0;
        vel.angular.y=0;
        vel.angular.z=0;   
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"turtlemove");
    turtlemove move;
    ros::spin();
    return 0;
}