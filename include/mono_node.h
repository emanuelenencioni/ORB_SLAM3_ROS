#include "common.h"

class Mono {
public:
    Mono(ORB_SLAM3::System* pSLAM, Common* comm):mpSLAM(pSLAM), common(comm){};

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
private:
    ORB_SLAM3::System* mpSLAM;
    Common* common;
};