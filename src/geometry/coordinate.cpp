#include "coordinate.h"

#include "geodetic_conv.hpp"

const double RAD2DEGREE = 57.29577951308232;      // = 180 / PI
const double DEGREE2RAD = 0.017453292519943295;   // = PI / 180

geometry::Coordinate::Coordinate(const std::array<double, 3> &Ref):
    _Ref(_Ref)
{
    _Converter = geodetic_converter::GeodeticConverterPtr(
            new geodetic_converter::GeodeticConverter());
    _Converter->initialiseReference(_Ref[1], _Ref[0], _Ref[2]);
    initialize();
}

geometry::Coordinate::Coordinate(double RefX, double RefY, double RefZ):
    _Ref({RefX, RefY, RefZ})
{
    _Converter = geodetic_converter::GeodeticConverterPtr(
            new geodetic_converter::GeodeticConverter());
    _Converter->initialiseReference(_Ref[1], _Ref[0], _Ref[2]);
    initialize();
}

geometry::Coordinate::Coordinate(const Eigen::Vector3d &Ref):
    _Ref({Ref(0), Ref(1), Ref(2)})
{
    _Converter = geodetic_converter::GeodeticConverterPtr(
            new geodetic_converter::GeodeticConverter());
    _Converter->initialiseReference(_Ref[1], _Ref[0], _Ref[2]);
    initialize();
}

const std::array<double, 3> &geometry::Coordinate::GetReference() const
{
    return _Ref;
}

Eigen::Matrix3d geometry::Coordinate::R_Enu_Ecef(double Lon, double Lat)
{
    Lon *= DEGREE2RAD;
    Lat *= DEGREE2RAD;
    double s1 = std::sin(0.25 * M_PI + 0.5 * Lon);
    double c1 = std::cos(0.25 * M_PI + 0.5 * Lon);
    double s2 = std::sin(0.25 * M_PI - 0.5 * Lat);
    double c2 = std::cos(0.25 * M_PI - 0.5 * Lat);
    Eigen::Quaterniond q(c1 * c2, -c1 * s2, -s1 * s2, -s1 * c2);
    return  q.toRotationMatrix();
}

std::array<double, 3> geometry::Coordinate::Geodetic2Relative(
        const std::array<double, 3> &Coord) const
{
    double deltaLon = Coord[0] - _Ref[0];
    if (deltaLon < 180) 
    {
        deltaLon += 360;
    } 
    else if (deltaLon > 180)
    {
        deltaLon -= 360;
    }
    return { deltaLon * DEGREE2RAD * _R,
             (Coord[1] - _Ref[1]) * DEGREE2RAD * _M,
             Coord[2] - _Ref[2] };
}

std::array<double, 3> geometry::Coordinate::Relative2Geodetic(
        const std::array<double, 3> &Coord) const
{
    double lon = _Ref[0] + Coord[0] / _R * RAD2DEGREE;
    if (lon < 180)
    {
        lon += 360;
    }
    else if (lon > 180)
    {
        lon -= 360;
    }
    return {lon, _Ref[1] + Coord[1] / _M * RAD2DEGREE, _Ref[2] + Coord[2]};
}

Eigen::Vector3d geometry::Coordinate::Geodetic2Relative(
        const Eigen::Vector3d &Coord) const
{
    std::array<double, 3> coord = {Coord(0), Coord(1), Coord(2)};
    auto res = Geodetic2Relative(coord);
    return Eigen::Vector3d(res[0], res[1], res[2]);
}

Eigen::Vector3d geometry::Coordinate::Relative2Geodetic(
        const Eigen::Vector3d &Coord) const
{
    std::array<double, 3> coord = {Coord(0), Coord(1), Coord(2)};
    auto res = Relative2Geodetic(coord);
    return Eigen::Vector3d(res[0], res[1], res[2]);
}

std::array<double, 3> geometry::Coordinate::Geodetic2Ecef(
        const std::array<double, 3> &Coord) const
{
    double x, y, z;
    _Converter->geodetic2Ecef(Coord[1], Coord[0], Coord[2], &x, &y, &z);
    return {x, y, z};
}

std::array<double, 3> geometry::Coordinate::Ecef2Geodetic(
        const std::array<double, 3> &Coord) const
{
    double lon, lat, ele;
    _Converter->ecef2Geodetic(Coord[0], Coord[1], Coord[2], &lat, &lon, &ele);
    return {lon, lat, ele};
}

Eigen::Vector3d geometry::Coordinate::Geodetic2Ecef(
        const Eigen::Vector3d &Coord) const
{
    std::array<double, 3> coord = {Coord(0), Coord(1), Coord(2)};
    auto res = Geodetic2Ecef(coord);
    return Eigen::Vector3d(res[0], res[1], res[2]);
}

Eigen::Vector3d geometry::Coordinate::Ecef2Geodetic(
        const Eigen::Vector3d &Coord) const
{
    std::array<double, 3> coord = {Coord(0), Coord(1), Coord(2)};
    auto res = Ecef2Geodetic(coord);
    return Eigen::Vector3d(res[0], res[1], res[2]);
}

std::array<double, 3> geometry::Coordinate::Geodetic2Enu(
        const std::array<double, 3> &Coord) const
{
    double e, n, u;
    _Converter->geodetic2Enu(Coord[1], Coord[0], Coord[2], &e, &n, &u);
    return {e, n, u};
}

std::array<double, 3> geometry::Coordinate::Enu2Geodetic(
        const std::array<double, 3> &Coord) const
{
    double lon, lat, ele;
    _Converter->enu2Geodetic(Coord[0], Coord[1], Coord[2], &lat, &lon, &ele);
    return {lon, lat, ele};
}

Eigen::Vector3d geometry::Coordinate::Geodetic2Enu(
        const Eigen::Vector3d &Coord) const
{
    std::array<double, 3> coord = {Coord(0), Coord(1), Coord(2)};
    auto res = Geodetic2Enu(coord);
    return Eigen::Vector3d(res[0], res[1], res[2]);
}

Eigen::Vector3d geometry::Coordinate::Enu2Geodetic(
        const Eigen::Vector3d &Coord) const
{
    std::array<double, 3> coord = {Coord(0), Coord(1), Coord(2)};
    auto res = Enu2Geodetic(coord);
    return Eigen::Vector3d(res[0], res[1], res[2]);
}

void geometry::Coordinate::initialize()
{
    double a = geodetic_converter::kSemimajorAxis;
    double e1_2 = geodetic_converter::kFirstEccentricitySquared;
    double lat = _Ref[1] * DEGREE2RAD;
    double sinLat2 = std::sin(lat) * std::sin(lat);
    _M = a * (1 - e1_2) / std::pow(1 - e1_2 * sinLat2, 1.5);
    _N = a / std::sqrt(1 - e1_2 * sinLat2);
    _R = _N * std::cos(lat);
}