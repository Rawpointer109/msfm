#pragma once

#include <map>
#include "basic_frame.h"

#include "global.h"

namespace msfm
{
namespace frame
{

class LidarFrame : public BasicFrame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LidarFrame();
    LidarFrame(size_t Timestamp, const Eigen::Matrix4f &Twf);

    void SetCloud(const CloudI::Ptr &Cloud, const std::vector<size_t> &Indices);
    void SetLaserIds(const std::vector<int8_t> &LaserIds);

    CloudI::Ptr GetFrameCloud() const;
    const std::vector<size_t> &GetIndices() const;
    
    // Index indicates inner-frame order
    inline bool PointAt(size_t Index, PointI &Out) const;  
    inline bool PointAt(uint16_t Row, uint16_t Col, PointI &Out) const;
    const std::map<int8_t, std::vector<size_t>> GetLabels();

    void CalculateIndexMat(bool RepeatScan, uint16_t Width, uint16_t Height,
                           float HorizontalInterval=0.3f, float VerticalInterval=0.3f);


private:
    // total point cloud
    CloudI::Ptr _Cloud;

    // index of each frame point in _Cloud
    std::vector<size_t> _Indices;

    // laser id of each frame point
    std::vector<int8_t> _LaserIds;

    // labels of each point
    std::vector<int8_t> _Labels;

    // fov slices count along horizontal and vertical directions
    uint16_t _Width, _Height;

    /**
     *  2d mat of frame point indices in _Cloud, 
     *  0 is used to indicates no-return point,
     *  which makes the first point of _Cloud will always be ignored.
     *  Is's acceptable considering total point cloud size.
     *  The advantage of using size_t rather than long is that,
     *  much more (about 2 times) points are able to be saved in _Cloud
     */
    Eigen::Matrix<size_t, -1, -1> _IndexMat;
};

} // namespace frame

} // namespace msfm