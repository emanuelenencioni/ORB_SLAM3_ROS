#include "common.h"
#include "json.h"

#include "ImuTypes.h"

class Stereo {
    public:
        Stereo(ORB_SLAM3::System* pSLAM, Common* comm): mpSLAM(pSLAM), common(comm){};

        /**
        * @brief callback for grabbing stereo images 
        * 
        * @param msgLeft left camera
        * @param msgRight right camera
        */
        void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);
    
    private:
        Common* common;
        ORB_SLAM3::System* mpSLAM;
};