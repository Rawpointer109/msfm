#include "coordinate.h"

#include "geometry/geodetic_conv.hpp"

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