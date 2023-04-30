#include "mono_node.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;
    ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::MONOCULAR;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    Common* comm = new Common(node_handler);
    ORB_SLAM3::System SLAM(comm->GetVocFile(), comm->GetSettingsFile(), sensor_type, comm->isPangolinEnabled());
    Mono node(&SLAM, comm);
    

    ros::Subscriber sub_img0 = node_handler.subscribe("/camera/image_raw", 1, &Mono::GrabImage, &node);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void Mono::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    Sophus::SE3f Twc = Tcw.inverse();

    ros::Time msg_time = msg->header.stamp;

    this->common->PublishPositionAsPoseStamped(Tcw, msg_time);
    this->common->PublishRosTrackedMapPoints(mpSLAM->GetAllMapPoints(), msg_time);
}