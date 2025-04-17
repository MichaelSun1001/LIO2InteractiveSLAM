#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "../include/jsonconvert.h"
#include "../include/tic_toc.h"

#define PI 3.1415926535

struct OdometryFrame;
class OdometrySet;

class odometry2graph
{
public:
    // odometry2graph(std::string _bag_path, std::string _odom_directory,
    //                float _downsample_resolution, bool _is_downsample,
    //                float _keyframe_delta_x, float _keyframe_delta_angle,
    //                int _friends_num);
    // odometry2graph(std::string _odom_directory,
    //                float _downsample_resolution, bool _is_downsample,
    //                float _keyframe_delta_x, float _keyframe_delta_angle,
    //                int _friends_num);
    odometry2graph();
    ~odometry2graph();

    enum Format
    {
        INTERACTIVE_SLAM_MODE = 0,
        STATION_MODE = 1
    };

    //加载.json文件
    bool load_config(std::string config_path);
    //根据.json文件加载和处理里程计信息
    void process();
    //保存地图，可以不给参数根据.json文件配置，也可以重新指定地图格式和保存路径
    void save_map();
    void save_map(Format _save_mode, std::string _save_path);
    void reset_keyframe_delta(float _keyframe_delta_x, float _keyframe_delta_angle);
    void reset_downsample(float _downsample_resolution, bool _is_downsample);
    void reset_friends_num(int _friends_num);

    void get_all_clouds(std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Isometry3d>> &all_clouds);

private:
    bool from_rosbag;
    std::string bag_path;
    std::string odom_directory;
    std::shared_ptr<OdometrySet> odometry_set;

    float downsample_resolution;
    bool is_downsample;
    float keyframe_delta_x, keyframe_delta_angle;
    int friends_num;
    Format save_mode;
    std::string save_path;
};
