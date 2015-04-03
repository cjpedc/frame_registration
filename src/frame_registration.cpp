#include "frame_registration.h"

namespace aick{

frame_registration::frame_registration(){

    ros::NodeHandle n_;

    image_rec_sub_ = n_.subscribe("/kinect2/depth_highres/points", 1, &frame_registration::cloud_imgrec, this);
    fpose_pub_ = n_.advertise<geometry_msgs::Point>("/frame_registration_pose", 1);

    return;
}

void frame_registration::cloud_imgrec(const sensor_msgs::PointCloud2::ConstPtr& input){

    counter_imgrec++;
    ROS_INFO("pointcloud in %i",counter_imgrec);

    pcl::fromROSMsg(*input, input_cloud);

    //Read data into images and save.
    if(save_data){

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

        bow();
    }


    return;
}

void frame_registration::bow(){
    printf("starting bag of words creation\n");

    string input = path_imgrec;             //Folder to load data from
    string output = path_bow;               //path to save output and initial name
    int nr_files = counter_imgrec;          //max number of files to investigate
    int step = 1;                           //See how many files to step
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

void frame_registration::images_fast_map(){

    printf("starting testing software2\n");
    printf("give path to files as input\n");
    string input = path_imgrec;

    Map3D * m = new Map3D();	//Create a standard map object
    m->setVerbose(true);		//Set the map to give text output

    m->loadCalibrationWords(path_bow,"orb", 500);	//set bag of words to orb 500 orb features from bow_path
    m->setFeatureExtractor(new OrbExtractor());		//Use orb features

    int max_points = 300;							//Number of keypoints used by matcher
    int nr_iter = 10;								//Number of iterations the matcher will run
    float shrinking = 0.7;							//The rate of convergence for the matcher
    float bow_threshold = 0.15;						//Bag of words threshold to avoid investigating bad matches
    float distance_threshold = 0.015;				//Distance threshold to discard bad matches using euclidean information.
    float feature_threshold = 0.15;					//Feature threshold to discard bad matches using feature information.

    m->setMatcher(new BowAICK(max_points, nr_iter,shrinking,bow_threshold,distance_threshold,feature_threshold));//Create a new matcher

    if(save_data){
        vector< RGBDFrame * > frames;
        for(int i = 1; i <=counter_imgrec ; i+=1){
            //printf("----------------------%i-------------------\nadding a new frame\n",i);

            //Get paths to image files
            char rgbbuf[512];
            char depthbuf[512];
            sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
            sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);

            //Add frame to map
            m->addFrame(string(rgbbuf) , string(depthbuf));

        }
    }else{

    }

    //m->addFrame(input_cloud);
    vector<Matrix4f> poses = m->estimate();	//Estimate poses for the frames using the map object.
    tf::Matrix3x3 rotationMat;
    rotationMat.setValue(poses.back()(0,0), poses.back()(0,1),poses.back()(0,2),
                         poses.back()(1,0), poses.back()(1,1),poses.back()(1,2),
                         poses.back()(2,0), poses.back()(2,1),poses.back()(2,2) );
    /*
    for(int r = 0; r <= 2; r++){
        for(int c = 0; c <= 2; c++){
            rotationMat(r,c) = poses.back()(r,c);
        }
    }
    */

    tf::Quaternion q;
    rotationMat.getRotation(q);

    f_pose.pose.position.x = poses.back()(0,3);
    f_pose.pose.position.y = poses.back()(1,3);
    f_pose.pose.position.z = poses.back()(2,3);
    f_pose.pose.orientation.x = q.x();
    f_pose.pose.orientation.y = q.y();
    f_pose.pose.orientation.z = q.z();
    f_pose.pose.orientation.w = q.w();
    f_pose.header.frame_id = "/camera_pose";

    fpose_pub_.publish(f_pose);


    m->savePCD("test.pcd");					//Saves a downsampled pointcloud with aligned data.

    //Print poses
    cout << "Poses:" << endl;
    for(unsigned int i = 0; i < poses.size(); i++){
        cout << poses.at(i) << endl << endl;
    }

    return;
}

}


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

