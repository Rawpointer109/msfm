#pragma once

#include <memory>

#include "map_interface.h"

typedef std::shared_ptr<msfm::MapInterface> MapInterfacePtr;

class Factory
{
public:
    Factory();
    static MapInterfacePtr CreateMapper();
};