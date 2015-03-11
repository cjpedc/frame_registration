#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <AICK.h>

class frame_registration
{

	ros::NodeHandle n_;


private:

public:


	frame_registration()
	{

	}

	~frame_registration()
	{

	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{

	}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "");
	frame_registration frame_registration_node;
    ros::Rate loop_rate(10);

  while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
  }

  return 0;
}
