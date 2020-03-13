#include "factory.h"

#include "map.h"

Factory::Factory()
{

}

MapInterfacePtr Factory::CreateMapper()
{
    return MapInterfacePtr(new msfm::Map());
}