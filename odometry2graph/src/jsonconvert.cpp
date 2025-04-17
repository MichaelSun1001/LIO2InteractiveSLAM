
#include "../include/jsonconvert.h"
namespace jsonconvert
{
    bool EigenVector3dFromJsonArray(Eigen::Vector3d &vec,
                                    const Json::Value &value)
    {
        if (value.size() != 3)
        {
            return false;
        }
        else
        {
            vec(0) = value[0].asDouble();
            vec(1) = value[1].asDouble();
            vec(2) = value[2].asDouble();
            return true;
        }
    }

    bool EigenVector3dToJsonArray(const Eigen::Vector3d &vec,
                                  Json::Value &value)
    {
        value.clear();
        value.append(vec(0));
        value.append(vec(1));
        value.append(vec(2));
        return true;
    }
    bool EigenVector4dFromJsonArray(Eigen::Vector4d &vec,
                                    const Json::Value &value)
    {
        if (value.size() != 4)
        {
            return false;
        }
        else
        {
            vec(0) = value[0].asDouble();
            vec(1) = value[1].asDouble();
            vec(2) = value[2].asDouble();
            vec(3) = value[3].asDouble();
            return true;
        }
    }

    bool EigenVector4dToJsonArray(const Eigen::Vector4d &vec,
                                  Json::Value &value)
    {
        value.clear();
        value.append(vec(0));
        value.append(vec(1));
        value.append(vec(2));
        value.append(vec(3));
        return true;
    }

    bool EigenMatrix3dFromJsonArray(Eigen::Matrix3d &mat,
                                    const Json::Value &value)
    {
        if (value.size() != 9)
        {
            return false;
        }
        else
        {
            for (int i = 0; i < 9; i++)
            {
                mat.coeffRef(i) = value[i].asDouble();
            }
            return true;
        }
    }

    bool EigenMatrix3dToJsonArray(const Eigen::Matrix3d &mat,
                                  Json::Value &value)
    {
        value.clear();
        for (int i = 0; i < 9; i++)
        {
            value.append(mat.coeffRef(i));
        }
        return true;
    }

    bool EigenMatrix4dFromJsonArray(Eigen::Matrix4d &mat,
                                    const Json::Value &value)
    {
        if (value.size() != 16)
        {
            return false;
        }
        else
        {
            for (int i = 0; i < 16; i++)
            {
                mat.coeffRef(i) = value[i].asDouble();
            }
            return true;
        }
    }

    bool EigenMatrix4dToJsonArray(const Eigen::Matrix4d &mat,
                                  Json::Value &value)
    {
        value.clear();
        for (int i = 0; i < 16; i++)
        {
            value.append(mat.coeffRef(i));
        }
        return true;
    }

    bool EigenMatrix6dFromJsonArray(Matrix6d &mat,
                                    const Json::Value &value)
    {
        if (value.size() != 36)
        {
            return false;
        }
        else
        {
            for (int i = 0; i < 36; i++)
            {
                mat.coeffRef(i) = value[i].asDouble();
            }
            return true;
        }
    }

    bool EigenMatrix6dToJsonArray(const Matrix6d &mat,
                                  Json::Value &value)
    {
        value.clear();
        for (int i = 0; i < 36; i++)
        {
            value.append(mat.coeffRef(i));
        }
        return true;
    }

    bool ReadJson(const std::string path, Json::Value &root, bool print_json)
    {

        std::cout << "\n\nload json from " << path << std::endl;

        std::ifstream file_in;
        file_in.open(path);
        if (!file_in.is_open())
        {
            std::cout << "open " << path << " failed" << std::endl;
            return false;
        }
        else
        {
            root.clear();
            file_in >> root;
            if (print_json)
            {
                std::cout << "load from file:\n"
                          << root << std::endl;
            }
        }
        return true;
    }
    bool WriteJson(const std::string path, Json::Value &root)
    {
        std::cout << "\n\nwrite MapInfo to json file" << std::endl;

        Json::StyledWriter style_writer;
        std::string str = style_writer.write(root);

        std::ofstream file_out;
        file_out.open(path);
        if (!file_out.is_open())
        {
            std::cout << "open file" << path << " failed!" << std::endl;
            return false;
        }
        {
            file_out << str;
            file_out.close();
        }
        return true;
    }
    std::string IntToStringWithLeadingZeros(int value, int totalLength)
    {
        std::ostringstream ss;
        ss << std::setw(totalLength) << std::setfill('0') << value;
        return ss.str();
    }

    bool EigenMatrix4dFromQuaterniondJsonArrary(Eigen::Matrix4d &mat, const Json::Value &quat_json)
    {
        if (quat_json.size() != 7)
        {
            std::cout << "quat_json size: " << quat_json.size() << " != 7" << std::endl;
            // open3d::utility::LogWarning("quat_json size: {} != 7", quat_json.size());
            return false;
        }
        Eigen::Quaterniond quat;
        Eigen::Vector3d trans;
        trans(0) = quat_json[0].asDouble();
        trans(1) = quat_json[1].asDouble();
        trans(2) = quat_json[2].asDouble();

        quat.x() = quat_json[3].asDouble();
        quat.y() = quat_json[4].asDouble();
        quat.z() = quat_json[5].asDouble();
        quat.w() = quat_json[6].asDouble();
        mat.setIdentity();
        mat.block<3, 3>(0, 0) = quat.matrix();
        mat.block<3, 1>(0, 3) = trans;
        return true;
    }
    bool EigenMatrix4dToQuaterniondJsonArrary(const Eigen::Matrix4d &mat, Json::Value &quat_json)
    {
        Eigen::Quaterniond quat;
        quat = mat.block<3, 3>(0, 0);
        Eigen::Vector3d trans = mat.block<3, 1>(0, 3);
        quat_json.clear();
        quat_json.append(trans(0));
        quat_json.append(trans(1));
        quat_json.append(trans(2));
        quat_json.append(quat.x());
        quat_json.append(quat.y());
        quat_json.append(quat.z());
        quat_json.append(quat.w());
        return true;
    }
} // namespace jsonconvert
