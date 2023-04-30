

using namespace std;

#include "rgbd_node.h"

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "RGBD");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;

    ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::RGBD;

    Common* comm = new Common(node_handler);//TODO da aggiustare
    ORB_SLAM3::System SLAM(comm->GetVocFile(), comm->GetSettingsFile(), sensor_type, comm->isPangolinEnabled());

    RGBDNode node(&SLAM, comm);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(node_handler, "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(node_handler, "/camera/depth_registered/image_raw", 100);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&RGBDNode::GrabRGBD,&node,_1,_2));

    

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void RGBDNode::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // ORB-SLAM3 runs in TrackRGBD()
    Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    Sophus::SE3f Twc = Tcw.inverse();

    ros::Time msg_time = cv_ptrRGB->header.stamp;

    //updating
    
    this->common->PublishPositionAsPoseStamped(Tcw, msg_time);
    this->common->PublishRosTrackedMapPoints(mpSLAM->GetAllMapPoints(), msg_time);
}