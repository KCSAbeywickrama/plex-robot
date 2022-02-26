#include "dashline.hpp"

namespace dashline
{
  double kp = 4;
  double ki = 0.001;
  double kd = 0.2;
  double p = 0;
  double i = 0;
  double d = 0;
  double error;
  double speed;
  double R_speed;
  double L_speed;
  double lasterror = 0;
  double etime = 0;
  double ttime = 0;
  int et = 0;
  int tc = 0;
  int pos;
  double t = 0.3;
  int s = 8;
  int act = 0;
  double values[8];
  double value[8];
  double value_max = 0;
  double value_min = 1000;
  int redBall = 0;
  bool end = 0;
  DistanceSensor *sensors[8];
  const char *sensor_names[8] = {"s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7"};

  Motor *lm;
  Motor *rm;

  void init(Robot *robot, int _redBall)
  {
    for (int i = 0; i < 8; i++)
    {
      sensors[i] = robot->getDistanceSensor(sensor_names[i]);
      sensors[i]->enable(TIME_STEP);
    }
    lm = robot->getMotor("leftMotor");
    rm = robot->getMotor("rightMotor");

    lm->setPosition(INFINITY);
    rm->setPosition(INFINITY);
    lm->setVelocity(0);
    rm->setVelocity(0);
    redBall = _redBall;
  }

  void sensor_check()
  {
    value_max = 0;
    value_min = 1000;
    act = 0;
    for (int i = 0; i < 8; i++)
    {
      value[i] = (sensors[i]->getValue());
      if (value[i] > value_max)
      {
        value_max = value[i];
      }
      if (value[i] < value_min)
      {
        value_min = value[i];
      }
    }
    if (abs(value_min - value_max) > 0.005)
    {
      t = (2.53268 - 1.23521 * pow(value_min, 0.239678) + 2.53268 - 1.23521 * pow(value_max, 0.239678)) / 2;
    }

    for (int i = 0; i < 8; i++)
    {
      values[i] = 1 - int((float)((2.53268 - 1.23521 * pow(value[i], 0.239678)) / t));
      act += values[i];
    }
  }

  void speedset()
  {
    lm->setVelocity(L_speed);
    rm->setVelocity(R_speed);
  }

  void endcheck(Robot *robot)
  {
    if (act > 5)
    {
      et += 1;
      etime = robot->getTime();
    }
    else if ((robot->getTime() - etime) > 0.2 && act < 5)
    {
      et = 0;
    }
    while (robot->step(TIME_STEP) != -1 && (robot->getTime() - etime) < 0.1 && etime != 0 && et > 6 && act < 8)
    {
      L_speed = PATH_BASE_SPEED;
      R_speed = PATH_BASE_SPEED;
      speedset();
      sensor_check();
      if (act == 0)
      {
        et = 0;
        end = 1;
      }
    }
  }

  void T_check(Robot *robot)
  {
    if (values[7] == 1 && values[0] == 0 && (robot->getTime() - ttime) < 0.1)
    {
      tc += 1;
      if (tc > 2 && redBall)
      {
        speedset();
        tc = 0;
        while (robot->step(TIME_STEP) != -1 && values[7] == 1)
        {
          sensor_check();
          L_speed = 0;
          R_speed = PATH_BASE_SPEED;
          speedset();
        }
      }
    }
    else
    {
      tc = 0;
    }
    ttime = robot->getTime();
  }

  void pid()
  {
    error = 0;
    pos = 0;
    if (act != 0)
    {
      for (int i = 0; i <= s; i++)
      {
        pos += values[i] * (i + 1);
      }
      error = (4.5 - ((float)pos / act));
    }
    if (act == 0 || act > 3)
    {
      error = 0;
    }

    p = error;
    i = i + error;
    d = error - lasterror;
    lasterror = error;
    speed = kp * p + ki * i + kd * d;

    L_speed = PATH_BASE_SPEED + speed;
    R_speed = PATH_BASE_SPEED - speed;

    if (L_speed > PATH_MAX_SPEED)
    {
      L_speed = PATH_MAX_SPEED;
    }
    if (R_speed > PATH_MAX_SPEED)
    {
      R_speed = PATH_MAX_SPEED;
    }
    if (L_speed < 0)
    {
      L_speed = 0;
    }
    if (R_speed < 0)
    {
      R_speed = 0;
    }
    speedset();
  }

  void follow(Robot *robot)
  {
    while (robot->step(TIME_STEP) != -1)
    {
      if (end)
      {
        L_speed = 0;
        R_speed = 0;
        speedset();
        break;
      }
      else
      {
        sensor_check();
        T_check(robot);
        pid();
        endcheck(robot);
      }
    }
  }
}
