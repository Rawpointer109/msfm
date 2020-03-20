#pragma once

#include "eigen3/Eigen/Dense"

namespace msfm
{
namespace geometry
{

class Triangulate
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Triangulate(const Eigen::Matrix<float, 3, 4> &M1,
                const Eigen::Vector2f &P1,
                const Eigen::Matrix<float, 3, 4> &M2,
                const Eigen::Vector2f &P2);
    
    bool LinearLs(Eigen::Vector3f &Out) const;
    bool LinearEigen(Eigen::Vector3f &Out) const;
    bool InterativeLs(Eigen::Vector3f &Out) const;
    bool InterativeEigen(Eigen::Vector3f &Out) const;

private:
    Eigen::Matrix<float, 3, 4> _M1, _M2;
    Eigen::Vector2f _P1, _P2;
};

} // namespace geometry

} // namespace msfm
