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
        _sub = _nh.subscribe("car_posearr", 10, &Leader_Tracker_Sub_Pub::Callback, this);
        _pub = _nh.advertise<geometry_msgs::PoseStamped>("leader_car_pose", 10);
    }

    // To track leader car and pubnish leader car pose 
    void Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        geometry_msgs::PoseArray obj_posearr = *msg;
        int size = obj_posearr.poses.size();
        if (size == 0)
        {
            ROS_INFO("No vehicle detected at this frame\n");
        }
        else
        {
            _tracker.ProcessMeasurement(obj_posearr);
            ROS_INFO("Vehicle detected at this frame\n");
            ROS_INFO("Convoy Leader Position x = %f\n", _tracker.kf_.x_[0]);
            ROS_INFO("Convoy Leader Position y = %f\n", _tracker.kf_.x_[1]);
            geometry_msgs::PoseStamped leader_car_pose;
            leader_car_pose.header = obj_posearr.header;
            leader_car_pose.pose.position.x = _tracker.kf_.x_[0];
            leader_car_pose.pose.position.y = _tracker.kf_.x_[1];
            _pub.publish(leader_car_pose);
        }
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