#include "dashline.hpp"

namespace dashline
{
  double kp =4;
  double ki = 0.001;
  double kd = 0.2;
  double p = 0;
  double i = 0;
  double d = 0;
  double error;
  double speed;
  double rs;
  double ls;
  double lasterror = 0;
  double etime = 0;
  double ttime = 0;
  int et = 0;
  int tc = 0;
  int pos;
  int t = 800;
  int s = 8;
  int act = 0;
  int values[8];
  int red = 0;
  bool end = 0;

  DistanceSensor *s1;
  DistanceSensor *s2;
  DistanceSensor *s3;
  DistanceSensor *s4;
  DistanceSensor *s5;
  DistanceSensor *s6;
  DistanceSensor *s7;
  DistanceSensor *s8;
  Motor *lm;
  Motor *rm;

  void init(Robot *robot)
  {
    s1 = robot->getDistanceSensor("s0");
    s2 = robot->getDistanceSensor("s1");
    s3 = robot->getDistanceSensor("s2");
    s4 = robot->getDistanceSensor("s3");
    s5 = robot->getDistanceSensor("s4");
    s6 = robot->getDistanceSensor("s5");
    s7 = robot->getDistanceSensor("s6");
    s8 = robot->getDistanceSensor("s7");
    lm = robot->getMotor("leftMotor");
    rm = robot->getMotor("rightMotor");

    lm->setPosition(INFINITY);
    rm->setPosition(INFINITY);
    lm->setVelocity(0);
    rm->setVelocity(0);
    s1->enable(TIME_STEP);
    s2->enable(TIME_STEP);
    s3->enable(TIME_STEP);
    s4->enable(TIME_STEP);
    s5->enable(TIME_STEP);
    s6->enable(TIME_STEP);
    s7->enable(TIME_STEP);
    s8->enable(TIME_STEP);
  }
  void sensor_check()
  {
    act = 0;

    values[0] = 1 - int((s1->getValue()) / t);
    values[1] = 1 - int((s2->getValue()) / t);
    values[2] = 1 - int((s3->getValue()) / t);
    values[3] = 1 - int((s4->getValue()) / t);
    values[4] = 1 - int((s5->getValue()) / t);
    values[5] = 1 - int((s6->getValue()) / t);
    values[6] = 1 - int((s7->getValue()) / t);
    values[7] = 1 - int((s8->getValue()) / t);
    
    for (int i = 0; i <= s; i++)
    {
      act += values[i];
    }
  }

  void speedset()
  {
    lm->setVelocity(ls);
    rm->setVelocity(rs);
  }

  void endcheck(Robot *robot)
  {
  if(act>5){
  et+=1;
  etime=robot->getTime();
  }
  else if((robot->getTime()-etime)>0.2 && act < 5){
  et=0;
  }
  while(robot->step(TIME_STEP) != -1 && (robot->getTime()-etime)<0.1 && etime!=0 && et>6 && act<8){
  ls=PATH_BASE_SPEED;
  rs=PATH_BASE_SPEED;
  speedset();
  sensor_check();
  if (act==0){
  et=0;
  end=1;
  }
  }
  }


void tcheck(Robot *robot)
  {
    if (values[7] == 1 && values[0] == 0)
    {
      if ((robot->getTime() - ttime) < 0.1)
      {
        tc += 1;
        if (tc > 2)
        {
          tc = 0;
          if (red)
          {
            while (robot->step(TIME_STEP) != -1 && values[7] == 1)
            {
              sensor_check();
              ls = 0;
              rs = PATH_BASE_SPEED;
              speedset();
            }
          }
        }
      }
      else
      {
        tc = 0;
      }
      ttime = robot->getTime();
    }
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
    else
    {
      error = 0;
    }
    if (act > 3)
    {
      error = 0;
    }
    
    
    p = error;
    i = i + error;
    d = error - lasterror;
    lasterror = error;
    speed = kp * p + ki * i + kd * d;
    speedset();
    
    ls = PATH_BASE_SPEED + speed;
    rs = PATH_BASE_SPEED - speed;

    if (ls > PATH_MAX_SPEED)
    {
      ls = PATH_MAX_SPEED;
    }
    if (rs > PATH_MAX_SPEED)
    {
      rs = PATH_MAX_SPEED;
    }
    if (ls < 0)
    {
      ls = 0;
    }
    if (rs < 0)
    {
      rs = 0;
    }
    speedset();
  }


  void dashfollow(Robot *robot)
  {
      while (robot->step(TIME_STEP) != -1)
      {
        if (end)
        {
          ls = 0;
          rs = 0;
          speedset();
          break;
        }
        else
        {
          sensor_check();
          tcheck(robot);
          pid();
          endcheck(robot);
        }
      }
    }
}