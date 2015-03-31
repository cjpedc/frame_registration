#include "frame_registration.h"
/*
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv.hpp>

#include <string.h>
*/


namespace aick{

frame_registration::frame_registration(){

    ros::NodeHandle n_;

    image_rec_sub_ = n_.subscribe("/kinect2/depth_highres/points", 1, &frame_registration::cloud_imgrec, this);

    return;
}

void frame_registration::cloud_imgrec(const sensor_msgs::PointCloud2::ConstPtr& input){

    counter_imgrec++;
    ROS_INFO("pointcloud in %i",counter_imgrec);
    pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
    pcl::fromROSMsg(*input, input_cloud);

    //Read data into images and save.

    int width = input_cloud.width;
    int height = input_cloud.height;

    IplImage * rgb_img							= cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_8U, 3);
    char * rgb_data								= (char *)(rgb_img->imageData);
    IplImage * depth_img						= cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_16U, 1);
    unsigned short * depth_data					= (unsigned short *)(depth_img->imageData);

    for(int w = 0; w < width; w++){
        for(int h = 0; h < height; h++){
            int ind = h*input_cloud.width + w;
            rgb_data[3*ind+0] = int(input_cloud.points[ind].b);
            rgb_data[3*ind+1] = int(input_cloud.points[ind].g);
            rgb_data[3*ind+2] = int(input_cloud.points[ind].r);
            depth_data[ind]   = (unsigned short)(5000*input_cloud.points[ind].z);
        }
    }

    char buf[1024];

    sprintf(buf,"%s/RGB%.10i.png",path_imgrec.c_str(),counter_imgrec);
    if(!cvSaveImage(buf,rgb_img)){printf("Could not save: %s\n",buf);}
    cvReleaseImage( &rgb_img);
    sprintf(buf,"%s/Depth%.10i.png",path_imgrec.c_str(),counter_imgrec);
    if(!cvSaveImage(buf,depth_img)){printf("Could not save: %s\n",buf);}
    cvReleaseImage( &depth_img );

    return;
}

void frame_registration::bow(){
    printf("starting testing software2\n");
    //printf("give path to files as input\n");
    string input = path_imgrec;             //Folder to load data from
    string output = path_bow;               //path to save output and initial name
    int nr_files = counter_imgrec;    //max number of files to investigate
    int step = 1;               //See how many files to step
    Map3D * m = new Map3Dbow(output);       //Create a bow map object
    m->setVerbose(true);                    //Set the map to give text output

    vector< RGBDFrame * > frames;
    for(int i = step; i <= nr_files; i+=step){
        printf("adding a new frame\n");

        //Get paths to image files
        char rgbbuf[512];
        char depthbuf[512];
        sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
        sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);

        printf("%i \n",i);
        //Add frame to map
        m->addFrame(string(rgbbuf) , string(depthbuf));
    }

    m->estimate();	//Estimate and save bag of words model.

    return;
}

}
/*
using namespace std;

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

*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "");

    aick::frame_registration aick_node;

    ros::Rate loop_rate(30);

  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }


  return 0;

}

