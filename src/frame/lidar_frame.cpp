#include "lidar_frame.h"

msfm::frame::LidarFrame::LidarFrame():
    BasicFrame()
{}

msfm::frame::LidarFrame::LidarFrame(size_t Timestamp, const Eigen::Matrix4f &Twf):
    BasicFrame(Timestamp, Twf)
{}

void msfm::frame::LidarFrame::SetCloud(
        const CloudI::Ptr &Cloud, const std::vector<size_t> &Indices)
{
    _Cloud = Cloud;
    _Indices = Indices;
}

void msfm::frame::LidarFrame::SetLaserIds(const std::vector<int8_t> &LaserIds)
{
    _LaserIds = LaserIds;
}

msfm::CloudI::Ptr msfm::frame::LidarFrame::GetFrameCloud() const
{
    CloudI::Ptr res(new CloudI());
    res->reserve(_Indices.size());
    for (size_t i : _Indices) 
    {
        res->push_back(_Cloud->at(i));
    }
    return res;
}

const std::vector<size_t> &msfm::frame::LidarFrame::GetIndices() const
{
    return _Indices;
}

inline bool msfm::frame::LidarFrame::PointAt(size_t Index, PointI &Out) const
{
    if (Index >= _Indices.size()) return false;
    Out = _Cloud->at(_Indices[Index]);
    return true;
}

inline bool msfm::frame::LidarFrame::PointAt(
        uint16_t Row, uint16_t Col, PointI &Out) const
{
    if (Row >= _Height || Col >= _Width || _IndexMat(Row, Col) == 0) return false;
    Out = _Cloud->at(_IndexMat(Row, Col));
    return true;
}

const std::map<int8_t, std::vector<size_t>> msfm::frame::LidarFrame::GetLabels()
{
    std::map<int8_t, std::vector<size_t>> res;
    for (size_t i = 0; i < _Indices.size(); ++i) 
    {
        int8_t label = _Labels[i];
        if (res.find(label) != res.end()) 
        {
            res[label].push_back(_Indices[i]);
        }
        else
        {
            res[label] = {_Indices[i]};
        }
    }
    return res;
}

void msfm::frame::LidarFrame::CalculateIndexMat(
        bool RepeatScan, uint16_t Width, uint16_t Height,
        float HorizontalInterval, float VerticalInterval)
{
    /* **********************************************************************
     *   TODO calculate 2d index matrix, for reconstruct fov image structure
     * **********************************************************************
     */ 
    std::cerr << "***  FUNCTION NOT READY!!  ***" << std::endl 
              << " in msfm::geometry::Triangulate::InterativeLs()" << std::endl;
}
