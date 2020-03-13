#include <iostream>
#include <string>

#include "map/factory.h"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: mapper path/to/task/file" << std::endl;
        exit(-1);
    }
    std::string taskFile = argv[1];
    auto mapper = Factory::CreateMapper();
    mapper->Mapping(taskFile);

    std::cout << "Process Finished!" << std::endl;
    return 0;
}
