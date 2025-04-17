#include "../include/odometry2graph.h"

struct OdometryFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<OdometryFrame>;

public:
    OdometryFrame(const std::string &raw_cloud_path, const Eigen::Isometry3d &pose, unsigned long stamp_sec, unsigned long stamp_usec) : raw_cloud_path(raw_cloud_path), cloud_(nullptr), pose(pose), stamp_sec(stamp_sec), stamp_usec(stamp_usec) {}

    OdometryFrame(const std::string &raw_cloud_path, const Eigen::Isometry3d &pose) : raw_cloud_path(raw_cloud_path), cloud_(nullptr), pose(pose), downsample_resolution(0.1f)
    {
        char underscore;
        std::stringstream sst(boost::filesystem::path(raw_cloud_path).filename().string());
        sst >> stamp_sec >> underscore >> stamp_usec;
    }

public:
    static OdometryFrame::Ptr load(const std::string &cloud_filename, const std::string &pose_filename)
    {
        std::ifstream ifs(pose_filename);
        if (!ifs)
        {
            std::cerr << "error : failed to load " << pose_filename << std::endl;
            return nullptr;
        }

        Eigen::Matrix4d mat;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                ifs >> mat(i, j);
            }
        }

        Eigen::Isometry3d pose(mat);

        return std::make_shared<OdometryFrame>(cloud_filename, pose);
    }

    static OdometryFrame::Ptr load(const std::string &cloud_filename, const Eigen::Isometry3d &pose, unsigned long stamp_sec, unsigned long stamp_usec)
    {
        return std::make_shared<OdometryFrame>(cloud_filename, pose, stamp_sec, stamp_usec);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud()
    {
        return cloud(downsample_resolution, is_downsample);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud(float downsample_resolution, bool is_downsample)
    {
        if (this->is_downsample)
        {
            if (is_downsample)
            {
                if (cloud_ == nullptr || std::abs(this->downsample_resolution - downsample_resolution) > 0.01f)
                {
                    cloud_ = load_clouds(raw_cloud_path, friends_filename, friends_pose);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

                    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
                    voxel_grid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
                    voxel_grid.setInputCloud(cloud_);
                    voxel_grid.filter(*downsampled);

                    this->downsample_resolution = downsample_resolution;
                    cloud_ = downsampled;
                }
            }
            else
            {
                this->downsample_resolution = downsample_resolution;
                this->is_downsample = is_downsample;
                cloud_ = load_clouds(raw_cloud_path, friends_filename, friends_pose);
            }
        }
        else
        {
            if (is_downsample)
            {
                if (cloud_ == nullptr)
                {
                    cloud_ = load_clouds(raw_cloud_path, friends_filename, friends_pose);
                }

                pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());

                pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
                voxel_grid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
                voxel_grid.setInputCloud(cloud_);
                voxel_grid.filter(*downsampled);

                this->downsample_resolution = downsample_resolution;
                this->is_downsample = is_downsample;
                cloud_ = downsampled;
            }
            else
            {
                if (cloud_ == nullptr)
                {
                    cloud_ = load_clouds(raw_cloud_path, friends_filename, friends_pose);
                }
                this->downsample_resolution = downsample_resolution;
            }
        }

        return cloud_;
    }

    void activate_cloud(float downsample_resolution, bool is_downsample)
    {
        cloud(downsample_resolution, is_downsample);
    }

    void deactivate_cloud()
    {
        if (cloud_ != nullptr)
        {
            cloud_.reset();
        }
        friends_filename.clear();
        friends_pose.clear();
    }

    void add_friend(std::string friend_cloud_path, Eigen::Isometry3d friend_pose)
    {
        friends_pose.push_back(friend_pose);
        friends_filename.push_back(friend_cloud_path);
    }

private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr load_clouds(const std::string &filename, const std::vector<std::string> &friends_filename, const std::vector<Eigen::Isometry3d> &friends_pose) const
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_transformed(new pcl::PointCloud<pcl::PointXYZI>());
        tmp = load_cloud(filename);
        *cloud += *tmp;

        for (int i = 0; i < friends_filename.size(); ++i)
        {
            tmp = load_cloud(friends_filename[i]);
            pcl::transformPointCloud(*tmp, *tmp_transformed, (pose.inverse() * friends_pose[i]).cast<float>());
            // pcl::transformPointCloud(*tmp, *tmp_transformed, (pose.inverse() * friends_pose[i]));
            *cloud += *tmp_transformed;
        }

        return cloud;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr load_cloud(const std::string &filename) const
    {
        std::string extension = boost::filesystem::path(filename).extension().string();
        if (extension == ".pcd")
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            // TicToc t_load;
            pcl::io::loadPCDFile(filename, *cloud);
            // std::cout << "load: " << t_load.toc() << std::endl;
            return cloud;
        }
        else if (extension == ".txt")
        {
            std::ifstream ifs(filename);
            if (!ifs)
            {
                std::cerr << "warning: failed to open " << filename << std::endl;
                return nullptr;
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
            while (!ifs.eof())
            {
                std::string line;
                std::getline(ifs, line);
                if (line.empty())
                {
                    continue;
                }

                std::stringstream sst(line);

                pcl::PointXYZI pt;
                sst >> pt.x >> pt.y >> pt.z >> pt.intensity;

                cloud->push_back(pt);
            }

            cloud->is_dense = false;
            cloud->width = cloud->size();
            cloud->height = 1;

            return cloud;
        }

        std::cerr << "unknown extension: " << extension << std::endl;
        std::cerr << "input file : " << filename << std::endl;
        abort();
        return nullptr;
    }

public:
    unsigned long stamp_sec;
    unsigned long stamp_usec;
    Eigen::Isometry3d pose;
    std::string raw_cloud_path;

    std::vector<Eigen::Isometry3d> friends_pose;
    std::vector<std::string> friends_filename;
    bool is_keyframe;
    int frame_id;
    int keyframe_id;

private:
    float downsample_resolution;
    bool is_downsample;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
};

/**
 * @brief Odometry frame set
 *
 */
class OdometrySet
{
public:
    OdometrySet(const std::string &directory)
    {
        load_directory(directory);
    }

    void select_keyframes(float keyframe_delta_x, float keyframe_delta_angle)
    {
        if (frames.empty())
        {
            std::cout << "no frames!" << std::endl;
            return;
        }

        int count = 0;

        keyframes.clear();
        keyframes.push_back(frames.front());
        frames.front()->keyframe_id = count;
        count++;
        for (const auto &frame : frames)
        {
            const auto &last_keyframe_pose = keyframes.back()->pose;
            const auto &current_frame_pose = frame->pose;

            Eigen::Isometry3d delta = last_keyframe_pose.inverse() * current_frame_pose;
            double delta_x = delta.translation().norm();
            double delta_angle = Eigen::AngleAxisd(delta.linear()).angle() / PI * 180;

            if (delta_x > keyframe_delta_x || delta_angle > keyframe_delta_angle)
            {
                keyframes.push_back(frame);
                frame->is_keyframe = true;
                frame->keyframe_id = count;
                count++;
            }
            else
            {
                frame->is_keyframe = false;
                frame->keyframe_id = -1;
            }
        }

        this->friends_num = 0;

        std::cout << "keyframes nums: " << keyframes.size() << std::endl;
    }

    void activate_keyframes(float downsample_resolution, bool is_downsample, int friends_num)
    {
        TicToc t0;
        for (const auto &frame : frames)
        {
            if (frame->is_keyframe)
            {
                std::cout << "current progress: " << frame->keyframe_id << " / " << keyframes.size() << std::endl;
                if (this->friends_num != friends_num)
                {
                    frame->deactivate_cloud();
                    for (int i = frame->frame_id - friends_num; i <= frame->frame_id + friends_num; ++i)
                    {
                        if (i < 0 || i >= frames.size() || i == frame->frame_id)
                            continue;

                        frame->add_friend(frames[i]->raw_cloud_path, frames[i]->pose);
                    }
                }
                frame->activate_cloud(downsample_resolution, is_downsample);
            }
            else
            {
                frame->deactivate_cloud();
            }
        }

        this->friends_num = friends_num;

        std::cout << "consume time: " << t0.toc() << " ms." << std::endl;
    }

    bool save(const std::string &dst_directory)
    {
        if (!boost::filesystem::exists(dst_directory))
            boost::filesystem::create_directories(dst_directory);
        if (keyframes.empty())
        {
            return false;
        }

        if (!save_graph(dst_directory + "/graph.g2o"))
        {
            return false;
        }

        if (!save_keyframes(dst_directory))
        {
            return false;
        }

        return true;
    }

    bool save_station(const std::string &dst_directory)
    {
        if (!boost::filesystem::exists(dst_directory))
            boost::filesystem::create_directories(dst_directory);
        if (keyframes.empty())
        {
            return false;
        }

        if (!save_keyframes_station(dst_directory))
        {
            return false;
        }

        return true;
    }

    void get_all_clouds(std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Isometry3d>> &all_clouds)
    {
        all_clouds.clear();
        for (int i = 0; i < keyframes.size(); i++)
        {
            all_clouds.push_back(std::make_pair(keyframes[i]->cloud(), keyframes[i]->pose));
        }

        return;
    }

private:
    void load_directory(const std::string &directory)
    {
        int count_id = 0;

        boost::filesystem::directory_iterator itr(directory);
        boost::filesystem::directory_iterator end;

        std::vector<std::string> filenames;
        for (itr; itr != end; itr++)
        {
            if (itr->path().extension() != ".pcd")
            {
                continue;
            }

            std::string odom_filename = itr->path().parent_path().string() + "/" + itr->path().stem().string() + ".odom";

            std::cout << "odom_filename: " << odom_filename << std::endl;

            if (!boost::filesystem::exists(odom_filename))
            {
                continue;
            }

            filenames.push_back(itr->path().stem().string());
        }

        std::sort(filenames.begin(), filenames.end());
        for (const auto &filename : filenames)
        {

            auto frame = OdometryFrame::load(directory + "/" + filename + ".pcd", directory + "/" + filename + ".odom");
            if (frame == nullptr)
            {
                continue;
            }

            frame->frame_id = count_id;
            count_id++;

            frames.push_back(frame);
        }
    }

    bool save_graph(const std::string &filename) const
    {

        std::ofstream ofs(filename);
        if (!ofs)
        {
            return false;
        }

        for (int i = 0; i < keyframes.size(); i++)
        {
            std::unique_ptr<g2o::VertexSE3> v(new g2o::VertexSE3());
            v->setEstimate(keyframes[i]->pose);

            ofs << "VERTEX_SE3:QUAT " << i << " ";
            v->write(ofs);
            ofs << std::endl;
        }
        ofs << "FIX 0" << std::endl;

        for (int i = 0; i < keyframes.size() - 1; i++)
        {
            const auto &delta_pose = keyframes[i]->pose.inverse() * keyframes[i + 1]->pose;
            std::unique_ptr<g2o::EdgeSE3> e(new g2o::EdgeSE3());
            e->setMeasurement(delta_pose);

            Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
            inf.block<3, 3>(0, 0) *= 10.0;
            inf.block<3, 3>(3, 3) *= 20.0;

            e->setInformation(inf);
            ofs << "EDGE_SE3:QUAT " << i << " " << i + 1 << " ";
            e->write(ofs);
            ofs << std::endl;
        }

        ofs.close();

        return true;
    }

    bool save_keyframes(const std::string &directory) const
    {
        for (int i = 0; i < keyframes.size(); i++)
        {
            std::string keyframe_directory = (boost::format("%s/%06d") % directory % i).str();
            boost::filesystem::create_directories(keyframe_directory);

            boost::filesystem::copy_file(keyframes[i]->raw_cloud_path, keyframe_directory + "/raw.pcd");
            pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *keyframes[i]->cloud());

            std::ofstream ofs(keyframe_directory + "/data");
            if (!ofs)
            {
                return false;
            }

            ofs << "stamp " << keyframes[i]->stamp_sec << " " << keyframes[i]->stamp_usec << std::endl;
            ofs << "estimate" << std::endl
                << keyframes[i]->pose.matrix() << std::endl;
            ofs << "odom " << std::endl
                << keyframes[i]->pose.matrix() << std::endl;
            ofs << "id " << i << std::endl;
        }

        return true;
    }

    bool save_keyframes_station(const std::string &directory) const
    {
        Json::Value collect_initial_pose_;
        for (int i = 0; i < keyframes.size(); i++)
        {
            pcl::io::savePCDFileBinary(directory + "/" + std::to_string(i) + ".pcd", *keyframes[i]->cloud());
            Json::Value json_mat;
            jsonconvert::EigenMatrix4dToJsonArray(keyframes[i]->pose.matrix(), json_mat);
            collect_initial_pose_[jsonconvert::IntToStringWithLeadingZeros(i, 3)]["pose_global_"] = json_mat;
        }

        std::string path_poses_initial = directory + "/poses_initial";
        if (!boost::filesystem::exists(path_poses_initial))
            boost::filesystem::create_directories(path_poses_initial);

        jsonconvert::WriteJson(path_poses_initial + "/poses_initial.json", collect_initial_pose_);

        return true;
    }

private:
    std::vector<OdometryFrame::Ptr> frames;
    std::vector<OdometryFrame::Ptr> keyframes;
    std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Isometry3d>> keyframes_cloud;
    int friends_num = 0;
};

odometry2graph::odometry2graph()
{
}

odometry2graph::~odometry2graph()
{
    std::cout << "bye" << std::endl;
}

bool odometry2graph::load_config(std::string config_path)
{
    Json::Value json_config;
    jsonconvert::ReadJson(config_path, json_config);
    std::cout << json_config << std::endl;
    from_rosbag = json_config["from_rosbag"].asBool();
    bag_path = json_config["bag_path"].asString();
    odom_directory = json_config["odom_directory"].asString();
    if (!boost::filesystem::exists(odom_directory))
    {
        std::cout << "odom_directory: " << odom_directory << " does not exist!" << std::endl;
        return false;
    }
    downsample_resolution = json_config["downsample_resolution"].asFloat();
    is_downsample = json_config["is_downsample"].asBool();
    keyframe_delta_x = json_config["keyframe_delta_x"].asFloat();
    keyframe_delta_angle = json_config["keyframe_delta_angle"].asFloat();
    friends_num = json_config["friends_num"].asInt();
    save_mode = static_cast<Format>(json_config["save_mode"].asInt());
    save_path = json_config["save_path"].asString();
    if (!boost::filesystem::exists(save_path))
    {
        std::cout << "save_path: " << save_path << " does not exist!" << std::endl;
        return false;
    }

    std::cout << "load config over" << std::endl;

    return true;
}

void odometry2graph::process()
{

    if (from_rosbag)
    {
        if (bag_path.empty())
        {
            std::cout << "no rosbag path!" << std::endl;
            return;
        }

        if (odom_directory.empty())
        {
            std::cout << "no odometry directory path!" << std::endl;
            return;
        }

        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Read);
        rosbag::View view(bag);

        for (const rosbag::MessageInstance &msg : view)
        {

            if (msg.getDataType() == "sensor_msgs/PointCloud2")
            {

                auto message = msg.instantiate<sensor_msgs::PointCloud2>();

                if (message != NULL)
                {

                    std::stringstream dst_filename;
                    dst_filename << odom_directory << "/" << message->header.stamp.sec << "_" << boost::format("%09d") % message->header.stamp.nsec << ".pcd";

                    std::cout << odom_directory << "/" << message->header.stamp.sec << "_" << boost::format("%09d") % message->header.stamp.nsec << ".pcd" << std::endl;

                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::fromROSMsg(*message, *cloud);

                    pcl::io::savePCDFileBinary(dst_filename.str(), *cloud);
                }
            }
            else if (msg.getDataType() == "nav_msgs/Odometry")
            {
                auto message = msg.instantiate<nav_msgs::Odometry>();

                if (message != NULL)
                {

                    std::stringstream dst_filename;
                    dst_filename << odom_directory << "/" << message->header.stamp.sec << "_" << boost::format("%09d") % message->header.stamp.nsec << ".odom";

                    std::cout << odom_directory << "/" << message->header.stamp.sec << "_" << boost::format("%09d") % message->header.stamp.nsec << ".odom" << std::endl;

                    const auto &pose = message->pose.pose;

                    Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
                    odom.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
                    odom.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).normalized().toRotationMatrix();

                    std::ofstream ofs(dst_filename.str());
                    ofs << odom.matrix();
                    ofs.close();
                }
            }
        }

        bag.close();
    }

    odometry_set.reset();
    odometry_set = std::make_shared<OdometrySet>(odom_directory);
    std::cout << "select keyframes start. waiting..." << std::endl;
    odometry_set->select_keyframes(keyframe_delta_x, keyframe_delta_angle);
    std::cout << "select keyframes over" << std::endl;
    std::cout << "activate keyframes start. waiting..." << std::endl;
    odometry_set->activate_keyframes(downsample_resolution, is_downsample, friends_num);
    std::cout << "activate keyframes over" << std::endl;
}

void odometry2graph::save_map()
{
    if (!odometry_set)
    {
        std::cout << "no data. process first!" << std::endl;
    }
    std::cout << "save start. waiting..." << std::endl;
    switch (save_mode)
    {
    default:
    case Format::INTERACTIVE_SLAM_MODE:
        odometry_set->save(save_path);
        break;

    case Format::STATION_MODE:
        odometry_set->save_station(save_path);
        break;
    }

    std::cout << "save over" << std::endl;
}

void odometry2graph::save_map(Format _save_mode, std::string _save_path)
{
    if (!odometry_set)
    {
        std::cout << "no data. process first!" << std::endl;
    }
    std::cout << "save start. waiting..." << std::endl;
    switch (_save_mode)
    {
    default:
    case Format::INTERACTIVE_SLAM_MODE:
        odometry_set->save(_save_path);
        break;

    case Format::STATION_MODE:
        odometry_set->save_station(_save_path);
        break;
    }

    std::cout << "save over" << std::endl;
}

void odometry2graph::reset_keyframe_delta(float _keyframe_delta_x, float _keyframe_delta_angle)
{
    keyframe_delta_x = _keyframe_delta_x;
    keyframe_delta_angle = _keyframe_delta_angle;
    std::cout << "reset. waiting..." << std::endl;
    odometry_set->select_keyframes(keyframe_delta_x, keyframe_delta_angle);
    odometry_set->activate_keyframes(downsample_resolution, is_downsample, friends_num);

    std::cout << "reset keyframe delta over" << std::endl;
}

void odometry2graph::reset_downsample(float _downsample_resolution, bool _is_downsample)
{
    downsample_resolution = _downsample_resolution;
    is_downsample = _is_downsample;
    std::cout << "reset. waiting..." << std::endl;
    odometry_set->activate_keyframes(downsample_resolution, is_downsample, friends_num);

    std::cout << "reset downsample over" << std::endl;
}

void odometry2graph::reset_friends_num(int _friends_num)
{
    friends_num = _friends_num;
    std::cout << "reset. waiting..." << std::endl;
    odometry_set->activate_keyframes(downsample_resolution, is_downsample, friends_num);

    std::cout << "reset friend num over" << std::endl;
}
void odometry2graph::get_all_clouds(std::vector<std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::Isometry3d>> &all_clouds)
{
    odometry_set->get_all_clouds(all_clouds);
    return;
}
