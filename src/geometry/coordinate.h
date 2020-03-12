#pragma once

#include <array>
#include <memory>
#include <vector>

#include "Eigen/Dense"

namespace geodetic_converter 
{
class GeodeticConverter;
typedef std::shared_ptr<GeodeticConverter> GeodeticConverterPtr;
}

namespace geometry
{

class Coordinate
{
public:
    Coordinate(const std::array<double, 3> &Ref);
    Coordinate(double RefX, double RefY, double RefZ);
    Coordinate(const Eigen::Vector3d &Ref);

    const std::array<double, 3> &GetReference() const;
    static Eigen::Matrix3d R_Enu_Ecef(double Lon, double Lat);

    std::array<double, 3> Geodetic2Relative(const std::array<double, 3> &Coord) const;
    std::array<double, 3> Relative2Geodetic(const std::array<double, 3> &Coord) const;
    Eigen::Vector3d Geodetic2Relative(const Eigen::Vector3d &Coord) const;
    Eigen::Vector3d Relative2Geodetic(const Eigen::Vector3d &Coord) const;

    std::array<double, 3> Geodetic2Ecef(const std::array<double, 3> &Coord) const;
    std::array<double, 3> Ecef2Geodetic(const std::array<double, 3> &Coord) const;
    Eigen::Vector3d Geodetic2Ecef(const Eigen::Vector3d &Coord) const;
    Eigen::Vector3d Ecef2Geodetic(const Eigen::Vector3d &Coord) const;

    std::array<double, 3> Geodetic2Enu(const std::array<double, 3> &Coord) const;
    std::array<double, 3> Enu2Geodetic(const std::array<double, 3> &Coord) const;
    Eigen::Vector3d Geodetic2Enu(const Eigen::Vector3d &Coord) const;
    Eigen::Vector3d Enu2Geodetic(const Eigen::Vector3d &Coord) const;

private:
    void initialize();

private:
    std::array<double, 3> _Ref;
    double _M, _N, _R;
    geodetic_converter::GeodeticConverterPtr _Converter;
};

}
