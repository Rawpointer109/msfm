#pragma once

#include <memory>

#include "basic_frame.h"

namespace msfm
{
namespace frame
{

struct Intrinsic
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Intrinsic> Ptr;
    Eigen::Matrix3f _K;      // fx, 0, cx, 0, fy, cy, 0, 0, 1
    int _Width, _Height;     // image width, image height
};


class CameraFrame : public BasicFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<CameraFrame> Ptr;
    CameraFrame();
    CameraFrame(size_t Timestamp, const Eigen::Matrix4f &Twf);

    void SetIntrinsic(const Intrinsic::Ptr &Intrinsic);
    int GetImageHeight() const;
    int GetImageWidth() const;
    Eigen::Matrix3f GetK() const;
    Eigen::Matrix<float, 3, 4> GetProjectionMatrix() const;

    /**
     * (X, Y, Z) to (u, v), in which u along image x axis
     * if point out of frustum, (-1, -1) is returned
     * */
    Eigen::Vector2f World2Pixel(const Eigen::Vector3f &Coord) const;
    
    /**
     *  (u, v) ==> (x, y, z), in which u along image x axis
     * */
    Eigen::Vector3f Pixel2Camera(const Eigen::Vector2f &Pixel, 
                                 float Depth = 1) const;
    
    /**
     *  (u, v) ==> (X, Y, Z), in which u along image x axis
     * */
    Eigen::Vector3f Pixel2World(const Eigen::Vector2f &Pixel, 
                                float Depth = 1) const;

    /**
     * May be improved by using more accurate triangulation algorithm
     */
    static bool Triangulate(const CameraFrame::Ptr &Cam1, 
                            const Eigen::Vector2f &Pixel1,
                            const CameraFrame::Ptr &Cam2,
                            const Eigen::Vector2f &Pixel2,
                            Eigen::Vector3f &Out);

private:
    Intrinsic::Ptr _Intrinsic;        // 1 intrinsic may be shared by multiple frames
};

}
}