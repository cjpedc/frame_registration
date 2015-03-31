#ifndef FRAME_REGISTRATION_HPP
#define FRAME_REGISTRATION_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>

#include <string.h>
#include "ekz.h"
#include "AICK.h"
#include "Map/Map3D.h"
#include "Map3Dbow.h"


namespace aick{

using namespace std;

class frame_registration{

public:
    frame_registration();
    string path_imgrec = "/home/tmrcv1/Desktop/images_test";
    string path_bow = "/home/tmrcv1/Desktop/images_test/bow_test/bow_test";
    int counter_imgrec = 0;
    void cloud_imgrec(const sensor_msgs::PointCloud2::ConstPtr& input);
    void bow();


private:

    ros::Subscriber image_rec_sub_;


};
}//namespace aick
#endif // FRAME_REGISTRATION_HPP
