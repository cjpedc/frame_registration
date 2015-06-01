#include "frame_registration.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/plane_refinement_comparator.h>

namespace aick{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
tf::Matrix3x3 rotationMat;
tf::Quaternion q;
int first, last;

frame_registration::frame_registration(){

    save_data = true;
    path_imgrec = "/home/tmrcv1/Desktop/images4";
    path_bow = "/home/tmrcv1/Desktop/images_test/bow_test/bow_base";
    counter_imgrec = 0;
    n_keymatches = 0;
    m = new Map3D();	//Create a standard map object
    poses.push_back(Matrix4f::Identity());
    prev_poses = poses;

    ros::NodeHandle n_;

    fpose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/frame_registration_pose/camera1", 1);

    return;
}

frame_registration::~frame_registration(){

    //    poses.clear();
    //    poses = m->estimate();
    //    m->savePCD("test.pcd");					//Saves a downsampled pointcloud with aligned data.

}

void frame_registration::bow(){ return;}

void frame_registration::images_fast_map(){

    string input = path_imgrec;
    //Map3D * m = new Map3D();	//Create a standard map object
    m->setVerbose(false);		//Set the map to give text output
    m->loadCalibrationWords(path_bow,"orb", 500);	//set bag of words to orb 500 orb features from bow_path
    m->setFeatureExtractor(new OrbExtractor());		//Use orb features

    int max_points = 300;					    //Number of keypoints used by matcher
    int nr_iter = 50;//8;							//Number of iterations the matcher will run
    float shrinking = 0.8;//0.7;				        //The rate of convergence for the matcher
    float bow_threshold = 0.15;//0.15;					//Bag of words threshold to avoid investigating bad matches
    float distance_threshold = 0.015;			//Distance threshold to discard bad matches using euclidean information.
    float feature_threshold = 0.15;//0.15;				//Feature threshold to discard bad matches using feature information.

    m->setMatcher(new BowAICK(max_points, nr_iter,shrinking,bow_threshold,distance_threshold,feature_threshold));//Create a new matcher

    ofstream out("position.txt");
    ofstream out2("matches.txt");
    ofstream out3("frames.txt");
    int gframes;

    vector< RGBDFrame * > frames;
    for(int i = first; i <=last ; i+=step){
        //printf("----------------------%i-------------------\nadding a new frame\n",i);

        //Get paths to image files
        char rgbbuf[512];
        char depthbuf[512];
        sprintf(rgbbuf,"%s/RGB%.10i.png",input.c_str(),i);
        sprintf(depthbuf,"%s/Depth%.10i.png",input.c_str(),i);

        //Add frame to map
        printf("----------------------%i-------------------\nadding a new frame\n",i);

        m->addFrame(string(rgbbuf) , string(depthbuf));
        counter_imgrec++;

        if(counter_imgrec > 1){


            n_keymatches = m->numberOfMatchesInLastFrame();
            cout << "MATCHES =" << n_keymatches << endl;

            if(n_keymatches < 80) // number of matches threshold
            {
                cout << "Too few feature matches, pose not estimated" << endl;

                cout << "Too few feature matches, removing last frame..." << endl;
                m->removeLastFrame();

            }else{
                poses = m->estimateCurrentPose(prev_poses);	//Estimate poses for the frames using the map object.
                prev_poses = poses;
                //poses = m->NEWestimate();	//Estimate poses for the frames using the map object.
                gframes = i;
            }

            // X-axis pointing right
            // Y-axis pointing down
            // Z-axis pointing forward

            // (Matlab) TM = makehgtfrom('xrotate',-pi/2)
            TM <<   1,  0, 0, 0,
                    0,  0, 1, 0,
                    0, -1, 0, 0,
                    0,  0, 0, 1 ;

            poseT = TM * poses.front();

            // (Matlab) TM = makehgtfrom('zrotate',-pi/2)
            TM <<   0, 1, 0, 0,
                   -1, 0, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1 ;

            poseT = TM * poseT;

            // TM reflected over X-Z plane to have Y-axis pointing right
            TM <<   1,  0, 0, 0,
                    0,  -1, 0, 0,
                    0,  0, 1, 0,
                    0,  0, 0, 1 ;

            poseT = TM * poseT;

            // X-axis pointing forward
            // Y-axis pointing right
            // Z-axis pointing up

            rotationMat.setValue(poseT(0,0), poseT(0,1),poseT(0,2),
                                 poseT(1,0), poseT(1,1),poseT(1,2),
                                 poseT(2,0), poseT(2,1),poseT(2,2) );

            //tf::Quaternion q;
            rotationMat.getRotation(q);
            f_pose.pose.position.x = poseT(0,3);
            f_pose.pose.position.y = poseT(1,3);
            f_pose.pose.position.z = poseT(2,3);
            f_pose.pose.orientation.x = q.x();
            f_pose.pose.orientation.y = q.y();
            f_pose.pose.orientation.z = q.z();
            f_pose.pose.orientation.w = q.w();
            f_pose.header.frame_id = "/camera1";
            f_pose.header.stamp.sec = sec_stamp_pdc;

            fpose_pub_.publish(f_pose);

            out << poseT(0,3) << ' ' << poseT(1,3) << ' ' << poseT(2,3) << '\n';
            out2 << n_keymatches << '\n';
            out3 << gframes << '\n';
        }

    }
    out.close();
    out2.close();
    out3.close();
    return;

}

} //end namespace aick


int main(int argc, char **argv)
{
    ros::init(argc, argv, "frame_registration");

    aick::frame_registration aick_node;

    aick_node.first = atoi(argv[1]);
    aick_node.last = atoi(argv[2]);
    aick_node.step = atoi(argv[3]);

    aick_node.images_fast_map();

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;

}
