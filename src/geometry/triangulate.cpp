#include "triangulate.h"

#include <iostream>

msfm::geometry::Triangulate::Triangulate(
        const Eigen::Matrix<float, 3, 4> &M1,
        const Eigen::Vector2f &P1,
        const Eigen::Matrix<float, 3, 4> &M2,
        const Eigen::Vector2f &P2):
        _M1(M1), _M2(M2), _P1(P1), _P2(P2)
{
}

bool msfm::geometry::Triangulate::LinearLs(Eigen::Vector3f &Out) const
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    Eigen::Matrix<float, 4, 3, Eigen::RowMajor> A;
    A << _P1(0) * _M1(2, 0) - _M1(0, 0), _P1(0) * _M1(2, 1) - _M1(0, 1), _P1(0) * _M1(2, 2) - _M1(0, 2),
         _P1(1) * _M1(2, 0) - _M1(1, 0), _P1(1) * _M1(2, 1) - _M1(1, 1), _P1(1) * _M1(2, 2) - _M1(1, 2),
         _P2(0) * _M2(2, 0) - _M2(0, 0), _P2(0) * _M2(2, 1) - _M2(0, 1), _P2(0) * _M2(2, 2) - _M2(0, 2),
         _P2(1) * _M2(2, 0) - _M2(1, 0), _P2(1) * _M2(2, 1) - _M2(1, 1), _P2(1) * _M2(2, 2) - _M2(1, 2);
    Eigen::Vector4f B;
    B << _M1(0, 3) - _P1(0) * _M1(2, 3), _M1(1, 3) - _P1(1) * _M1(2, 3),
         _M2(0, 3) - _P2(0) * _M2(2, 3), _M2(1, 3) - _P2(1) * _M2(2, 3);
    try
    {
        Out = A.colPivHouseholderQr().solve(B);
        return true;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl 
                  << " in msfm::geometry::Triangulate::LinearLs()" << std::endl;
        return false;
    }
}

bool msfm::geometry::Triangulate::LinearEigen(Eigen::Vector3f &Out) const
{
    std::cerr << "***  FUNCTION NOT READY!!  ***" << std::endl 
              << " in msfm::geometry::Triangulate::LinearEigen()" << std::endl;
    return false;
}

bool msfm::geometry::Triangulate::InterativeLs(Eigen::Vector3f &Out) const
{
    std::cerr << "***  FUNCTION NOT READY!!  ***" << std::endl 
              << " in msfm::geometry::Triangulate::InterativeLs()" << std::endl;
    return false;
}

bool msfm::geometry::Triangulate::InterativeEigen(Eigen::Vector3f &Out) const
{
    std::cerr << "***  FUNCTION NOT READY!!  ***" << std::endl 
              << " in msfm::geometry::Triangulate::InterativeEigen()" << std::endl;
    return false;
}