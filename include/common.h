#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <driverless_msgs/bounding_boxes.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "include/System.h"
#include "include/ImuTypes.h"
class Common{
    public:
        /**
         * @brief Construct a new Common object
         * 
         * @param nh the ros node handler
         */
        Common(ros::NodeHandle& nh);
        ~Common();

        /**
         * @brief publish the  pointcloud of the mapPoints
         * 
         * @param map_points the array of point to publish
         * @param msg_time the msg timestamp
         */
        void PublishRosTrackedMapPoints(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time);

        /**
         * @brief tranform the pose to a different frame using tf
         * 
         * @param T_SE3f the pose of the camera
         * @param frame_id the frame id of the camera
         * @param child_frame_id the new frame of the pose
         * @param msg_time the msg timestamp
         */
        void PublishRosTfTransform(Sophus::SE3f T_SE3f, string frame_id, string child_frame_id, ros::Time msg_time);
        
        /**
         * @brief function that publish the camera pose
         *  
         * @param position the current pose of the camera
         * @param current_frame_time ros's time
         * @param map_frame_id_param map frame id
         * @param to_target if true, it will try to transform the position to the target frame, BE SURE that the transform in tf exist to the target frame!
         */
        void PublishPositionAsPoseStamped(Sophus::SE3f position, ros::Time current_frame_time, bool to_target = false);
        
        tf2::Transform TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target);
        
        tf2::Transform TransformFromMat (Sophus::SE3f T_SE3f);

        /**
         * @brief coordinate transformation from SE3f to tf::Transform
         * 
         * @param T_SE3f 
         * @return tf::Transform 
         */
        tf::Transform SE3FToTfTransform(Sophus::SE3f T_SE3f);
        

        std::string GetWorldFrameId();

        std::string GetCamFrameId();

        bool isPangolinEnabled();

        std::string GetVocFile();

        std::string GetSettingsFile();
        
        std::string GetPathSMFile();
    private:
        /**
         * @brief Set the up ros publishers object
         * 
         * @param nh the node_handler
         */
        void SetupRosPublishers(ros::NodeHandle& nh);

        void Init(ros::NodeHandle& nh);

        /**
         * @brief transformation from array of MapPoint to sensor_msgs::PointCloud2 
         * 
         * @param map_points the array of mapPoints
         * @param msg_time the message timestamp
         * @return sensor_msgs::PointCloud2 
         */
        sensor_msgs::PointCloud2 TrackedMapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time);

        std::string worldFrameId, camFrameId, imuFrameId, targetFrameId, parentFrameId, vocFile, settingsFile, pathToSaveMapFile;
        boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> tfListener;
        ros::Publisher posePub, mapPointsPub;
        bool enablePangolin;
};