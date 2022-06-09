#include "ros/ros.h"
#include "std_msgs/String.h"
#include <DataHandlerClass.h>
#include <ti_mmwave_rospkg/RadarScan.h>

 
void chatterCallback(const ti_mmwave_rospkg::RadarScan::ConstPtr &msg)
{
	if (msg->point_id == 0){
		ROS_INFO("______________________________________________________");
	}
	ROS_INFO("id: %d x: %f y: %f z: %f v: %f",msg->point_id, msg->x, msg->y, msg->z, msg->velocity);

	/*std_msgs::String pub;
	pub=[msg->x, msg->y, msg->z];
	ros::NodeHandle r;
	ros::Publisher point_pub = r.adverties<std_msgs::String>("point",1000);
	point_pub.publish(pub);*/
}

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "radar_scan_sub_pub");

	
	ros::NodeHandle nh;

	
	 
	ros::Subscriber radar_scan_sub = nh.subscribe("/ti_mmwave/radar_scan", 100, chatterCallback);

	
	ros::spin();

	return 0;
}