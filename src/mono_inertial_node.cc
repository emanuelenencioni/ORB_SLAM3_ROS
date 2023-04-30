#include "mono_inertial_node.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::IMU_MONOCULAR;
    
    Common* comm = new Common(node_handler);
    ORB_SLAM3::System SLAM(comm->GetVocFile(), comm->GetSettingsFile(), sensor_type, comm->isPangolinEnabled());

    MonoInertial node(&SLAM, comm);

    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &MonoInertial::GrabImu, &node); 
    ros::Subscriber sub_img0 = node_handler.subscribe("/camera/image_raw", 100, &MonoInertial::GrabImage, &node);

    std::thread sync_thread(&MonoInertial::SyncWithImu, &node);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}

void MonoInertial::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!img0Buf.empty())
        img0Buf.pop();
    img0Buf.push(img_msg);
    mBufMutex.unlock();
}

cv::Mat MonoInertial::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonoInertial::SyncWithImu()
{
    while(1)
    {
        if (!img0Buf.empty()&&!this->imuBuf.empty())
        {
            cv::Mat im;
            double tIm = 0;

            tIm = img0Buf.front()->header.stamp.toSec();
            if(tIm>this->imuBuf.back()->header.stamp.toSec())
                continue;
            
            this->mBufMutex.lock();
            im = GetImage(img0Buf.front());
            ros::Time msg_time = img0Buf.front()->header.stamp;
            img0Buf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            this->mBufMutex.lock();
            if (!this->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!this->imuBuf.empty() && this->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    double t = this->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(this->imuBuf.front()->linear_acceleration.x, this->imuBuf.front()->linear_acceleration.y, this->imuBuf.front()->linear_acceleration.z);
                    
                    cv::Point3f gyr(this->imuBuf.front()->angular_velocity.x, this->imuBuf.front()->angular_velocity.y, this->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                    this->imuBuf.pop();
                }
            }
            this->mBufMutex.unlock();

            // Main algorithm runs here
            Sophus::SE3f Tcw = mpSLAM->TrackMonocular(im, tIm, vImuMeas);
            Sophus::SE3f Twc = Tcw.inverse();
            
            this->common->PublishPositionAsPoseStamped(Tcw, msg_time);
            this->common->PublishRosTrackedMapPoints(mpSLAM->GetAllMapPoints(), msg_time);
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

void MonoInertial::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}