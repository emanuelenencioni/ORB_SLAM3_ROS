#include "common.h"

class MonoInertial
{
    public:
        MonoInertial(ORB_SLAM3::System* pSLAM, Common* comm): mpSLAM(pSLAM), common(comm){};

        void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
        void GrabImage(const sensor_msgs::ImageConstPtr& msg);
        cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
        void SyncWithImu();
    private:
        queue<sensor_msgs::ImageConstPtr> img0Buf;
        std::mutex mBufMutex;
        ORB_SLAM3::System* mpSLAM;
        Common* common;


        queue<sensor_msgs::ImuConstPtr> imuBuf;
};