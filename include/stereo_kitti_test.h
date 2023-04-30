#include <std_msgs/String.h>

#include "common.h"
#include "json.h"
#include "ImuTypes.h"


class StereoKitti{
    public:
        StereoKitti(ORB_SLAM3::System* pSLAM, Common* comm, const std::string path):mpSLAM(pSLAM), common(comm), pathToFilename(path), saved(false), msgCounter(0) {};
        /**
        * @brief callback for grabbing stereo images 
        * 
        * @param msgLeft left camera
        * @param msgRight right camera
        */
        void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight);
        /**
         * @brief callback for calling saveTrackKitti when the sequence is finished
         * 
         * @param msg if the msg isn't "save", then the map will not be saved
         */
        void SaveTrack(const std_msgs::String msg);

    private:
        ORB_SLAM3::System* mpSLAM;
        Common* common;
        std::string pathToFilename;
        bool saved;
        int msgCounter;
};