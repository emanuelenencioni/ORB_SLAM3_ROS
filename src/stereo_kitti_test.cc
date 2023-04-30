#include "stereo_kitti_test.h"

using json = nlohmann::json;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;
   
     // Create SLAM system. It initializes all system threads and gets ready to process frames.
    Common* common = new Common(node_handler);
    ORB_SLAM3::System SLAM(common->GetVocFile(), common->GetSettingsFile(), ORB_SLAM3::System::STEREO, common->isPangolinEnabled());
    
    StereoKitti node(&SLAM, common, common->GetPathSMFile());
    message_filters::Subscriber<sensor_msgs::Image> left_sub(node_handler, "/camera/left/image_raw", 5000);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(node_handler, "/camera/right/image_raw", 5000);
    ros::Subscriber save_sub = node_handler.subscribe("/load_image/save", 50, &StereoKitti::SaveTrack, &node);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5000), left_sub, right_sub);
    sync.registerCallback(boost::bind(&StereoKitti::GrabStereo,&node,_1,_2));
    std::cout<<"setup finished"<<std::endl;
    ros::spin();

    // Stop all threads
    ros::shutdown();

    return 0;
}

void StereoKitti::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Main algorithm runs here
    Sophus::SE3f Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    Sophus::SE3f Twc = Tcw.inverse();
    msgCounter += 1;
    ros::Time msg_time = cv_ptrLeft->header.stamp;
    //this->common->PublishRosTfTransform(Tcw, common->GetWorldFrameId(), "base_link", msg_time);
    this->common->PublishPositionAsPoseStamped(Tcw, msg_time, false);
    //this->common->PublishRosTrackedMapPoints(mpSLAM->GetAllMapPoints(), msg_time);
}
void StereoKitti::SaveTrack(const std_msgs::String msg) {
    if(msg.data.compare("save") == 0 && !(this->saved)){
        usleep(10e6);
        std::cout<<"total image received: "<<msgCounter<<std::endl;
        saved = true;
        this->mpSLAM->Shutdown();
        this->mpSLAM->SaveTrajectoryKITTI(this->pathToFilename + "CameraTrajectory.txt");
        ros::shutdown();
    }
    std::cout<<"fatto"<<endl;
}   