#include "mosaic.hpp"
#include "navigate.hpp"
#include "mscstps.hpp"
#include "arm.hpp"

namespace mscstps
{
    void pickAndInsert(Robot *robot)
    {
        navigate::navigateObject(robot);

        int object = navigate::detectObject(robot);
        if (object != OBJ_CYLNDR)
        {
            mosaic::lookFromLeft(robot);
            object = navigate::detectObject(robot);
            if (object != OBJ_CYLNDR)
            {
                mosaic::lookFromRight(robot);
                object = navigate::detectObject(robot);
            }
        }

        arm::gripObject(robot, 0.001, 0);
        arm::raise(robot, 1.4);

        mosaic::preAlignKeyHole(robot);

        if (object == OBJ_CYLNDR)
            mosaic::goYellow2Cylinder(robot);
        else if (object == OBJ_BOX)
            mosaic::goYellow2Box(robot);
        else
            cout << "onject detect error" << endl;

        arm::shoot(robot);
    }

    void run(Robot *robot)
    {
        navigate::init(robot);

        pickAndInsert(robot);
        mosaic::goHole2Cyan(robot);
        pickAndInsert(robot);
    }
}

// mosaic::lookFromLeft(robot);
// navigate::detectObject(robot);
// mosaic::delay(robot, 10);
// mosaic::lookFromRight(robot);
// navigate::detectObject(robot);