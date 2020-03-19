#include "camera_frame.h"


msfm::frame::CameraFrame::CameraFrame():
        BasicFrame()
{
}

msfm::frame::CameraFrame::CameraFrame(size_t Timestamp, 
                                      const Eigen::Matrix4f &Twf):
        BasicFrame(Timestamp, Twf)
{
}

void msfm::frame::CameraFrame::SetIntrinsic(const Intrinsic::Ptr &Intrinsic)
{
    _Intrinsic = Intrinsic;
}

int msfm::frame::CameraFrame::GetImageHeight() const
{
    return _Intrinsic->_Height;
}

int msfm::frame::CameraFrame::GetImageWidth() const
{
    return _Intrinsic->_Width;
}

Eigen::Matrix3f msfm::frame::CameraFrame::GetK() const
{
    return _Intrinsic->_K;
}

Eigen::Matrix<float, 3, 4> msfm::frame::CameraFrame::GetProjectionMatrix() const
{
    Eigen::Matrix<float, 3, 4> Rt = GetTwf().inverse().topLeftCorner(3, 4);
    return _Intrinsic->_K * Rt;
}

Eigen::Vector2f msfm::frame::CameraFrame::World2Pixel(
        const Eigen::Vector3f &Coord) const
{
    Eigen::Vector3f pCam = World2Frame(Coord);
    pCam /= pCam[2];
    Eigen::Vector3f pixel = _Intrinsic->_K * pCam;
    if (pixel[1] < 0 || pixel[1] >= _Intrinsic->_Width 
            || pixel[0] < 0 || pixel[1] >= _Intrinsic->_Height) 
    {
        return Eigen::Vector2f(-1, -1);
    } 
    else
    {
        return Eigen::Vector2f(pixel[1], pixel[0]);
    }
}

Eigen::Vector3f msfm::frame::CameraFrame::Pixel2Camera(
        const Eigen::Vector2f &Pixel, 
        float Depth) const
{
    // x = (u - cx) * z / fx
    float x = (Pixel[1] - _Intrinsic->_K(0, 2)) * Depth / _Intrinsic->_K(0, 0);
    // y = (v - cy) * z / fy
    float y = (Pixel[0] - _Intrinsic->_K(1, 2)) * Depth / _Intrinsic->_K(1, 1);
    return Eigen::Vector3f(x, y, Depth);
}

Eigen::Vector3f msfm::frame::CameraFrame::Pixel2World(
        const Eigen::Vector2f &Pixel, 
        float Depth) const
{
    Eigen::Vector3f pCam = Pixel2Camera(Pixel, Depth);
    return Frame2World(pCam);
}

bool msfm::frame::CameraFrame::Triangulate(
        const CameraFrame::Ptr &Cam1, 
        const Eigen::Vector2f &Pixel1,
        const CameraFrame::Ptr &Cam2,
        const Eigen::Vector2f &Pixel2,
        Eigen::Vector3f &Out)
{
    Eigen::Vector3f lhs = Cam1->Pixel2Camera(Pixel1);
    Eigen::Vector3f rhs = Cam2->Pixel2Camera(Pixel2);
    Eigen::Vector3f lhsRay = Cam1->GetTwf().topLeftCorner(3, 3) * lhs;
    Eigen::Vector3f rhsRay = Cam2->GetTwf().topLeftCorner(3, 3) * rhs;
    float cos = lhsRay.dot(rhsRay) / (lhsRay.norm() * rhsRay.norm());
    float threshold = 0.99f;
    if (cos < 0 || cos > threshold)  // lines of sight are parallel or diverging
    {
        return false;
    }
    Eigen::Matrix4f x1, x2;
    x1 << -1,  0, lhs(0), 0,
           0, -1, lhs(1), 0,
           0,  0,      0, 0,
           0,  0,      0, 0;
    x2 <<  0,  0,      0, 0,
           0,  0,      0, 0,      
          -1,  0, rhs(0), 0,
           0, -1, rhs(1), 0;
    Eigen::Matrix4f A = x1 * Cam1->GetTwf().inverse() +
                        x2 * Cam2->GetTwf().inverse();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinV);
    Eigen::Vector4f p = svd.matrixV().col(3);
    p /= p(3);
    Out = p.head(3);
    return Cam1->World2Frame(Out)(2) > 0 && Cam2->World2Frame(Out)(2) > 0;
}