#pragma once

#include "map_interface.h"
#include "json.hpp"

namespace msfm
{

class Map : public MapInterface
{
public:
    Map();
    ~Map() override;
    
    void Mapping(const std::string &TaskFile) override;

private:
    void loadCalibration();
    void loadData();

private:
    nlohmann::json _Task;

};

} // namespace msfm
