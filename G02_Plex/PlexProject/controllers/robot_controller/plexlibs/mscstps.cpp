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

        if (object == OBJ_CYLNDR)
            cout << "cylender detected" << endl;
        else if (object == OBJ_BOX)
            cout << "box detected" << endl;

        navigate::checkNear(robot);
        arm::gripObject(robot, 0.001, 0);
        arm::loose(robot);
        navigate::checkNear(robot);
        arm::gripObject(robot, 0.001, 0);
        arm::raise(robot, 1.4);

        mosaic::preAlignKeyHole(robot);

        if (object == OBJ_CYLNDR)
            mosaic::goYellow2Cylinder(robot);
        else if (object == OBJ_BOX)
            mosaic::goYellow2Box(robot);
        else
            cout << "object detect error" << endl;

        arm::shoot(robot);
    }

    void run(Robot *robot, int redBall)
    {
        mosaic::goWall2MagentaEnc(robot);
        navigate::init(robot);
        pickAndInsert(robot);
        mosaic::goHole2Cyan(robot);
        pickAndInsert(robot);
        mosaic::turnRight(robot);
        navigate::navigateBall(robot, redBall);
        arm::gripObject(robot, 0.001, OBJ_BALL);
        arm::raise(robot, 1.2);
        mosaic::exit(robot);
        arm::raise(robot, 0.0);
        mosaic::goFront(robot, 140);
    }
}

// mosaic::lookFromLeft(robot);
// navigate::detectObject(robot);
// mosaic::delay(robot, 10);
// mosaic::lookFromRight(robot);
// navigate::detectObject(robot);