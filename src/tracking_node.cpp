#include "ros/ros.h"
#include "Tracker.cpp"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

using namespace std;


class Leader_Tracker_Sub_Pub
{
public:
    //Default Constructor
    Leader_Tracker_Sub_Pub()
    {
        _sub = _nh.subscribe("misc_points", 10, &Leader_Tracker_Sub_Pub::Callback, this);
        _pub = _nh.advertise<geometry_msgs::PoseStamped>("leader_car", 10);
    }

    // To track leader car and pubnish leader car pose 
    void Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        //extract the postarr from PointCloud2
        sensor_msgs::PointCloud2 objpc = *msg;
        //geometry_msgs::PoseArray posearr_msg;
        //posearr_msg.header = objpc.header;
        ROS_INFO("inside callback");     

//        _tracker.ProcessMeasurement(posearr_msg);

        //extract the poseStamped from _tracker._x
//        geometry_msgs::PoseStamped pose_msg;

        // publish the pose of convoy leader to topid "leader_car"
 //       _pub.publish(pose_msg);
    }

private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    ros::Subscriber _sub;
    Tracker _tracker;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracking_node");
    Leader_Tracker_Sub_Pub leader_tracker_sub_pubOBJ;
    ros::spin();
    return 0;
}