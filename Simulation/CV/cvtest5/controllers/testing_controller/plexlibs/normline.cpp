#include "normline.hpp"

namespace normline
{
    double kp = 1.2;
    double ki = 0.001;
    double kd = 0.2;
    bool insquare = false;
    bool firststage = false;
    bool end = false;

    double p = 0;
    double i = 0;
    double d = 0;
    double error;
    double speed;
    double rs;
    double ls;
    int pos;
    double t = 500;
    int s = 8;
    int act = 0;
    int v1;
    int v2;
    int v3;
    int v4;
    int v5;
    int v6;
    int v7;
    int v8;
    double value[8];
    double values[8];
    int red = 1;
    double lasterror = 0;
    DistanceSensor *sensors[8];
    const char* sensor_names[8]={"s0","s1","s2","s3","s4","s5","s6","s7"};
    double value_max=0;
    double value_min=1000; 

    Motor *lm;
    Motor *rm;

    void init(Robot *robot)
    {
        
        for(int i=0;i<8;i++){
        sensors[i]=robot->getDistanceSensor(sensor_names[i]);
        sensors[i]->enable(TIME_STEP);
        }

        lm = robot->getMotor("leftMotor");
        rm = robot->getMotor("rightMotor");

        lm->setPosition(INFINITY);
        rm->setPosition(INFINITY);
        lm->setVelocity(0);
        rm->setVelocity(0);
    }

    void squre_check()
    {
        int count = 0;
        for (int i = 0; i <= s; i++)
        {
            if (i)
            {
                count += 1;
            }
            else
            {
                break;
            }
        }
        if (((count) < 3) && (act < 3) && (count == act))
        {
            insquare = false;
        }
    }

    void sensor_check()
    
    {
    value_max=0;
    value_min=1000;  
    act = 0;
    for(int i =0; i<8;i++){
    value[i]=(sensors[i]->getValue());
    //std::cout << 2.53268-1.23521*pow(value[i],0.239678) << std::endl;
    if(value[i]>value_max){
    value_max=value[i];
    }
    if(value[i]<value_min){
    value_min=value[i];
    }
    
    }
    if(abs(value_min-value_max)>0.005){
    t=(2.53268-1.23521*pow(value_min,0.239678)+2.53268-1.23521*pow(value_max,0.239678))/2;
    }
    
    std::cout << "**************" << std::endl;
    std::cout <<"t = "<< t << std::endl;
    std::cout << value_min<< std::endl;
    std::cout << value_max<< std::endl;
    std::cout << "--------------" << std::endl;
    
    
    
    for(int i =0; i<8;i++){
    values[i] = 1- int((float)((2.53268-1.23521*pow(value[i],0.239678)) / t));
    std::cout << values[i] << std::endl;
    act+=values[i];
    }
    /*std::cout << "**************" << std::endl;
    std::cout << t << std::endl;
    std::cout << "--------------" << std::endl;*/
  }

    void speedset()
    {
        lm->setVelocity(ls);
        rm->setVelocity(rs);
    }

    void endcheck(Robot *robot)
    {

        for (int i = 0; i < 40; i++)
        {
            sensor_check();
            if (act == 0)
            {
                robot->step(TIME_STEP);
            }
            else
            {
                break;
            }
            if (i > 38)
            {
                end = 1;
                ls = 0;
                rs = 0;
                speedset();
            }
            else
            {
                end = 0;
            }
        }
    }

    void err()
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
    }

    void pid()
    {
        err();
        p = error;
        i = i + error;
        d = error - lasterror;
        lasterror = error;
        speed = kp * p + ki * i + kd * d;

        ls = NORM_BASE;
        rs = NORM_BASE;
        ls = NORM_BASE + speed;
        rs = NORM_BASE - speed;

        if (ls > NORM_MAX)
        {
            ls = NORM_MAX;
        }
        if (rs > NORM_MAX)
        {
            rs = NORM_MAX;
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

    void follow(Robot *robot)
    {
        while (robot->step(TIME_STEP) != -1)
        {
            sensor_check();
            if (insquare)
            {
                squre_check();
                ls = NORM_BASE;
                rs = NORM_BASE;
            }
            else
            {
                if (end)
                {
                    break;
                }
                pid();
                endcheck(robot);
            }
        }
    }
}//pp