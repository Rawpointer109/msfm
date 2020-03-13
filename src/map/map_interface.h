#pragma once

#include <string>

namespace msfm
{

class MapInterface
{
public:
    virtual ~MapInterface();
    virtual void Mapping(const std::string &TaskFile) = 0;
};

}