#include "common.h"


Common::Common(ros::NodeHandle& nh){
    this->SetupRosPublishers(nh);
    this->Init(nh);
}

void Common::SetupRosPublishers(ros::NodeHandle &node_handler)
{
    this->posePub = node_handler.advertise<geometry_msgs::PoseStamped>("orb_slam3/camera_pose", 1);

    this->mapPointsPub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3/map", 1);
    
    this->tfBuffer.reset(new tf2_ros::Buffer);
    this->tfListener.reset(new tf2_ros::TransformListener(*tfBuffer));
}

void Common::Init(ros::NodeHandle &node_handler){
    std::string node_name = ros::this_node::getName();
    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", vocFile, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settingsFile, "file_not_set");

    if (vocFile == "file_not_set" || settingsFile == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        throw std::runtime_error("Please provide voc_file and settings_file in the launch file");
    } 
    node_handler.param<std::string>(node_name + "/world_frame_id", worldFrameId, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", camFrameId, "camera");
    node_handler.param<std::string>(node_name + "/target_frame_id", targetFrameId, "base_link");
    node_handler.param<std::string>(node_name + "/parent_frame_id", parentFrameId, "base_link");
    
    node_handler.param<bool>(node_name + "/enable_pangolin", enablePangolin, true);
    node_handler.param<std::string>(node_name + "/path", pathToSaveMapFile, "");
}

void Common::PublishRosTfTransform(Sophus::SE3f T_SE3f, string frame_id, string child_frame_id, ros::Time msg_time)
{
    tf2::Transform tf_transform = TransformFromMat(T_SE3f);
    tf2::Transform fin_tf_transform = TransformToTarget(tf_transform, camFrameId, child_frame_id);
    static tf2_ros::TransformBroadcaster tf_broadcaster;

    geometry_msgs::TransformStamped transf_stamped;
    transf_stamped.transform.rotation.w = fin_tf_transform.getRotation().getW();
    transf_stamped.transform.rotation.x = fin_tf_transform.getRotation().getX();
    transf_stamped.transform.rotation.y = fin_tf_transform.getRotation().getY();
    transf_stamped.transform.rotation.z = fin_tf_transform.getRotation().getZ();

    transf_stamped.transform.translation.x = fin_tf_transform.getOrigin().getX();
    transf_stamped.transform.translation.y = fin_tf_transform.getOrigin().getY();
    transf_stamped.transform.translation.z = fin_tf_transform.getOrigin().getZ();
    transf_stamped.header.frame_id = frame_id;
    transf_stamped.child_frame_id = child_frame_id;
    transf_stamped.header.stamp = msg_time;
    tf_broadcaster.sendTransform(transf_stamped);
}

void Common::PublishRosTrackedMapPoints(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = TrackedMapPointsToPointCloud(map_points, msg_time);
    
    mapPointsPub.publish(cloud);
}


sensor_msgs::PointCloud2 Common::TrackedMapPointsToPointCloud(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time) {
    const int num_channels = 4; // x y z rgb

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = this->worldFrameId;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z", "rgb"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);

    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i]) //check if the mapPoint is at least observed "min_observations_per_point" times.
        {
            Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();
            int color = 16777215;
            tf::Vector3 point_translation(P3Dw.z(), -P3Dw.x(), -P3Dw.y()); //we do this point transformation to make alligned to rviz axis.
            
            float data_array[num_channels] = {
                point_translation.x(),
                point_translation.y(),
                point_translation.z(),
                color
            };

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

tf::Transform Common::SE3FToTfTransform(Sophus::SE3f T_SE3f) {
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );

    tf::Vector3 t_tf(
        t_vec(0),
        t_vec(1),
        t_vec(2)
    );

    return tf::Transform(R_tf, t_tf);
}


void Common::PublishPositionAsPoseStamped(Sophus::SE3f position, ros::Time current_frame_time, bool to_target) {
    

    
    tf2::Transform tf_position = TransformFromMat(position);
    tf2::Transform tf_position_target;
    tf2::Stamped<tf2::Transform> tf_position_target_stamped;

    if(to_target && this->targetFrameId.compare("") != 0) {
        tf_position_target = TransformToTarget(tf_position, this->camFrameId, this->targetFrameId);
        tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position_target, current_frame_time, this->worldFrameId);// Make transform from camera frame to target frame
    }
    else
        tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position, current_frame_time, this->worldFrameId);

    // Make message
    
    
    geometry_msgs::PoseStamped pose_msg;
    tf2::toMsg(tf_position_target_stamped, pose_msg);
    posePub.publish(pose_msg);
}

tf2::Transform Common::TransformToTarget (tf2::Transform tf_in, std::string frame_in, std::string frame_target) {
    // Transform tf_in from frame_in to frame_target
    tf2::Transform tf_map2orig = tf_in;
    tf2::Transform tf_orig2target;
    tf2::Transform tf_map2target;

    tf2::Stamped<tf2::Transform> transformStamped_temp;
    try {
    // Get the transform from camera to target
    geometry_msgs::TransformStamped tf_msg = tfBuffer->lookupTransform(frame_in, frame_target, ros::Time(0));
    // Convert to tf2
    tf2::fromMsg(tf_msg, transformStamped_temp);
    tf_orig2target.setBasis(transformStamped_temp.getBasis());
    tf_orig2target.setOrigin(transformStamped_temp.getOrigin());

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
        tf_orig2target.setIdentity();
    }

    // Transform from map to target
    tf_map2target = tf_map2orig * tf_orig2target;
    return tf_map2target;
}

tf2::Transform Common::TransformFromMat (Sophus::SE3f T_SE3f) {
    Eigen::Matrix3f rotation = T_SE3f.rotationMatrix();
    Eigen::Vector3f translation = T_SE3f.translation();

    tf2::Matrix3x3 tf_camera_rotation (
                                    rotation(0, 0), rotation(0, 1), rotation(0, 2),
                                    rotation(1, 0), rotation(1, 1), rotation(1, 2),
                                    rotation(2, 0), rotation(2, 1), rotation(2, 2)
                                    );

    tf2::Vector3 tf_camera_translation (translation(0), translation(1), translation(2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                        -1, 0, 0,
                                        0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
    tf_camera_rotation = tf_camera_rotation.transpose();
    tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
    tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
    tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

    return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

std::string Common::GetWorldFrameId() {
    return worldFrameId;
}

std::string Common::GetCamFrameId() {
    return camFrameId;
}

bool Common::isPangolinEnabled() {
    return this->enablePangolin;
}

std::string Common::GetVocFile() {
    return this->vocFile;
}

std::string Common::GetSettingsFile() {
    return this->settingsFile;
}
std::string Common::GetPathSMFile() {
    return this->pathToSaveMapFile;
}


Common::~Common(){}