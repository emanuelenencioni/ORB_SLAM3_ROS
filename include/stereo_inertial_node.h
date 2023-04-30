#include "common.h"

class StereoInertial {
public:
    

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    StereoInertial(ORB_SLAM3::System* pSLAM, Common* comm): mpSLAM(pSLAM),  common(comm){};

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
private:
    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft,mBufMutexRight;
    ORB_SLAM3::System* mpSLAM;
    Common* common;

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};