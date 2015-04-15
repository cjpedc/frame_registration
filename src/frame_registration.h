#ifndef FRAME_REGISTRATION_HPP
#define FRAME_REGISTRATION_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Eigen>

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
    bool save_data;
    frame_registration();
    string path_imgrec;
    string path_bow;

    geometry_msgs::PoseStamped f_pose;

    int counter_imgrec;
    int n_keymatches;
    double sec_stamp_pdc;
    void cloud_imgrec(const sensor_msgs::PointCloud2::ConstPtr& input);
    void bow();
    void images_fast_map();
    ~frame_registration();


private:

    ros::Subscriber image_rec_sub_;
    ros::Publisher fpose_pub_;
    Map3D * m;	//Create a standard map object
    vector<Matrix4f> poses;
    vector<Matrix4f> prev_poses;

};
}//namespace aick
#endif // FRAME_REGISTRATION_HPP
