#pragma once

#include <cstdint>

#include "eigen3/Eigen/Dense"


namespace msfm
{
namespace frame
{

class BasicFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BasicFrame();
    BasicFrame(size_t Timestamp, const Eigen::Matrix4f &Twf);

    std::size_t GetTimestamp() const;
    void SetTimestamp(size_t Timestamp);

    const Eigen::Matrix4f &GetTwf() const;
    void SetTwf(const Eigen::Matrix4f &Twf);

    const Eigen::Matrix4f &GetTfi() const;
    void SetTfi(const Eigen::Matrix4f &Tfi);

    const Eigen::Vector3f &GetPoseError() const;
    void SetPoseError(const Eigen::Vector3f &PoseError);

    /**
     *  Conversion between world coordinate system and frame coordinate system
     * */
    Eigen::Vector3f World2Frame(const Eigen::Vector3f &Coord) const;
    Eigen::Vector3f Frame2World(const Eigen::Vector3f &Coord) const;

private:
    std::size_t _Timestamp;        // timestamp in microseconds
    Eigen::Matrix4f _Twf;          // T_world_frame
    Eigen::Matrix4f _Tfi;          // T_frame_imu, eg. T_lidar_imu or T_camera_imu
    Eigen::Vector3f _PoseError;    // pose error in 3d cartesian direction
};

}
}