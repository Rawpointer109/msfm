#include "basic_frame.h"

msfm::frame::BasicFrame::BasicFrame():
        _Timestamp(0), 
        _Twf(Eigen::Matrix4f::Identity())
{}

msfm::frame::BasicFrame::BasicFrame(size_t Timestamp, const Eigen::Matrix4f &Twf):
        _Timestamp(Timestamp),
        _Twf(Twf)
{}

std::size_t msfm::frame::BasicFrame::GetTimestamp() const
{
    return _Timestamp;
}

void msfm::frame::BasicFrame::SetTimestamp(size_t Timestamp)
{
    _Timestamp = _Timestamp;
}

const Eigen::Matrix4f &msfm::frame::BasicFrame::GetTwf() const
{
    return _Twf;
}

void msfm::frame::BasicFrame::SetTwf(const Eigen::Matrix4f &Twf)
{
    _Twf = Twf;
}

const Eigen::Matrix4f &msfm::frame::BasicFrame::GetTfi() const
{
    return _Tfi;
}

void msfm::frame::BasicFrame::SetTfi(const Eigen::Matrix4f &Tfi)
{
    _Tfi = Tfi;
}

const Eigen::Vector3f &msfm::frame::BasicFrame::GetPoseError() const
{
    return _PoseError;
}

void msfm::frame::BasicFrame::SetPoseError(const Eigen::Vector3f &PoseError)
{
    _PoseError = PoseError;
}

Eigen::Vector3f msfm::frame::BasicFrame::World2Frame(
        const Eigen::Vector3f &Coord) const
{
    Eigen::Matrix4f Tfw = _Twf.inverse();
    return Tfw.topLeftCorner(3, 3) * Coord + Tfw.topRightCorner(3, 1);
}

Eigen::Vector3f msfm::frame::BasicFrame::Frame2World(
        const Eigen::Vector3f &Coord) const
{
    return _Twf.topLeftCorner(3, 3) * Coord + _Twf.topRightCorner(3, 1);
}