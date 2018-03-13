#include "ros/ros.h"
#include "Tracker.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

using namespace std;
using namespace cv;


class Leader_Tracker
{
    Tracker _tracker;
    ros::Publisher _pub;
public:
    //Default Constructor
    Leader_Tracker(ros::Publisher pub)
    {
        _pub = pub;
    }
    // Todo convert pointcloud2 to PoseArray
    void Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        //extract the postarr from PointCloud2
        geometry_msgs::PoseArray posearr_msg = 1;

        _tracker.ProcessMeasurement(posearr_msg);

        //extract the poseStamped from _tracker._x
        geometry_msgs::PoseStamped pose_msg = 1;

        // publish the pose of convoy leader to topid "leader_car"
        _pub.publish(pose_msg);
    }
private:
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracking_node");
    ros::NodeHandle nh;

    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("leader_car", 10);
    Leader_Tracker leader_tracker(pub);
    ros::Subscriber sub = nh.subscribe("car_points", 10, Leader_Tracker::Callback);
    ros::spin();
    return 0;
}