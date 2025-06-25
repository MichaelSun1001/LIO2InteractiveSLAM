#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <jsoncpp/json/json.h>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace jsonconvert
{
    bool EigenVector3dFromJsonArray(Eigen::Vector3d &vec,
                                    const Json::Value &value);
    bool EigenVector3dToJsonArray(const Eigen::Vector3d &vec,
                                  Json::Value &value);

    bool EigenVector4dFromJsonArray(Eigen::Vector4d &vec,
                                    const Json::Value &value);
    bool EigenVector4dToJsonArray(const Eigen::Vector4d &vec,
                                  Json::Value &value);

    bool EigenMatrix3dFromJsonArray(Eigen::Matrix3d &mat,
                                    const Json::Value &value);
    bool EigenMatrix3dToJsonArray(const Eigen::Matrix3d &mat,
                                  Json::Value &value);

    /// @brief 将json转为matrix4d
    /// @param mat
    /// @param value
    /// @return
    bool EigenMatrix4dFromJsonArray(Eigen::Matrix4d &mat,
                                    const Json::Value &value);

    /// @brief 将matrix4d转为json
    /// @param mat
    /// @param value
    /// @return
    bool EigenMatrix4dToJsonArray(const Eigen::Matrix4d &mat,
                                  Json::Value &value);

    bool EigenMatrix6dFromJsonArray(Matrix6d &mat,
                                    const Json::Value &value);
    bool EigenMatrix6dToJsonArray(const Matrix6d &mat,
                                  Json::Value &value);

    /// @brief 将json中的四元数(x, y, z, q_x, q_y, q_z, q_w)转为matrix4d
    /// @param mat
    /// @param quat_json
    /// @return
    bool EigenMatrix4dFromQuaterniondJsonArrary(Eigen::Matrix4d &mat,
                                                const Json::Value &quat_json);

    /// @brief 将matrix4d转为(x, y, z, q_x, q_y, q_z, q_w)放进json
    /// @param mat
    /// @param quat_json
    /// @return
    bool EigenMatrix4dToQuaterniondJsonArrary(const Eigen::Matrix4d &mat,
                                              Json::Value &quat_json);

    std::string IntToStringWithLeadingZeros(int value, int totalLength);
    bool ReadJson(const std::string path, Json::Value &value, bool print_json = false);
    bool WriteJson(const std::string path, Json::Value &value);

} // namespace jsonconvert
