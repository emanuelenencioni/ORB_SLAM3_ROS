#include "stereo_node.h"


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
    
    Common* common = new Common(node_handler);
    ORB_SLAM3::System SLAM(common->GetVocFile(), common->GetSettingsFile(), ORB_SLAM3::System::STEREO, common->isPangolinEnabled());
    Stereo node(&SLAM, common);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(node_handler, "/camera/left/image_raw", 50);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(node_handler, "/camera/right/image_raw", 50);
    
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&Stereo::GrabStereo,&node,_1,_2));
    
    ros::spin();

    // Stop all threads
    
    ros::shutdown();

    return 0;
}



void Stereo::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
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

    ros::Time msg_time = cv_ptrLeft->header.stamp;
    this->common->PublishRosTfTransform(Tcw, common->GetWorldFrameId(), "base_link", msg_time);
    this->common->PublishPositionAsPoseStamped(Tcw, msg_time, true);
    this->common->PublishRosTrackedMapPoints(mpSLAM->GetAllMapPoints(), msg_time);
}