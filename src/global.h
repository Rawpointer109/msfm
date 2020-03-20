#pragma once

#include "pcl-1.9/pcl/point_cloud.h"
#include "pcl-1.9/pcl/point_types.h"


namespace msfm
{

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointXYZINormal PointN;
typedef pcl::PointCloud<PointN> CloudN;

} // namespace msfm
