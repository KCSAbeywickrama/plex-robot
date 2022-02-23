#include "mosaic.hpp"
#include "navigate.hpp"
#include "mscstps.hpp"

namespace mscstps
{
    void run(Robot *robot)
    {
        navigate::init(robot);
        navigate::navigateObject(robot);
        navigate::detectObject(robot);
        mosaic::lookFromLeft(robot);
        navigate::detectObject(robot);
        mosaic::lookFromRight(robot);
        navigate::detectObject(robot);
    }
}