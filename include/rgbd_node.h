#include "common.h"

class RGBDNode {
public:
    RGBDNode(ORB_SLAM3::System* pSLAM, Common* comm):mpSLAM(pSLAM), common(comm){};
    
    /**
    * @brief callback of the subscrier
    * 
    * @param msgRGB the rgb image
    * @param msgD the depth image
    */
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

private:
    ORB_SLAM3::System* mpSLAM;
    Common* common;
};