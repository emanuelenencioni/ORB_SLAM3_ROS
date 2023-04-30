#include "stereo_inertial_node.h"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo_Inertial");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;
     Common* common = new Common(node_handler);
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(common->GetVocFile(), common->GetSettingsFile(), ORB_SLAM3::System::IMU_STEREO, common->isPangolinEnabled());

    StereoInertial node(&SLAM, common);

    // Maximum delay, 5 seconds * 200Hz = 1000 samples
    ros::Subscriber sub_imu = node_handler.subscribe("/imu", 1000, &StereoInertial::GrabImu, &node); 
    ros::Subscriber sub_img_left = node_handler.subscribe("/camera/left/image_raw", 100, &StereoInertial::GrabImageLeft, &node);
    ros::Subscriber sub_img_right = node_handler.subscribe("/camera/right/image_raw", 100, &StereoInertial::GrabImageRight, &node);


    std::thread sync_thread(&StereoInertial::SyncWithImu, &node);

    ros::spin();

    return 0;
}

void StereoInertial::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void StereoInertial::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat StereoInertial::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
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

void StereoInertial::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!this->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            if(tImLeft>this->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            ros::Time msg_time = imgLeftBuf.front()->header.stamp;
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            this->mBufMutex.lock();
            if(!this->imuBuf.empty())
            {
                // Load imu measurements from buffer
                //vImuMeas.clear();
                while(!this->imuBuf.empty() && this->imuBuf.front()->header.stamp.toSec()<=tImLeft)
                {
                    double t = this->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(this->imuBuf.front()->linear_acceleration.x, this->imuBuf.front()->linear_acceleration.y, this->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(this->imuBuf.front()->angular_velocity.x, this->imuBuf.front()->angular_velocity.y, this->imuBuf.front()->angular_velocity.z);
                    
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));

                    this->imuBuf.pop();
                }
            }
            this->mBufMutex.unlock();
            // ORB-SLAM3 runs in TrackStereo()
            Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
            Sophus::SE3f Twc = Tcw.inverse();
            
            this->common->PublishPositionAsPoseStamped(Twc, msg_time);
            this->common->PublishRosTrackedMapPoints(mpSLAM->GetAllMapPoints(), msg_time);
            
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void StereoInertial::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    
    return;
}