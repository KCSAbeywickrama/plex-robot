#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define time 16
#define time1 16
#define max 6.28
#define base 4

double kp=1.2;
double ki=0.001;
double kd=0.2;
bool insquare = false;
bool firststage = false;
bool end= false;
 
double p=0;
double i=0;
double d=0;
double error;
double speed;
double rs;
double ls;
int pos;
int t=800;
int s = 8;
int act=0;
int v1;
int v2;
int v3;
int v4;
int v5;
int v6;
int v7;
int v8;
int values [8];
int red=1;
  
double lasterror = 0;

using namespace webots;

Robot *robot = new Robot();
DistanceSensor *s1=robot->getDistanceSensor("s0");
DistanceSensor *s2=robot->getDistanceSensor("s1");
DistanceSensor *s3=robot->getDistanceSensor("s2"); 
DistanceSensor *s4=robot->getDistanceSensor("s3");
DistanceSensor *s5=robot->getDistanceSensor("s4");
DistanceSensor *s6=robot->getDistanceSensor("s5"); 
DistanceSensor *s7=robot->getDistanceSensor("s6"); 
DistanceSensor *s8=robot->getDistanceSensor("s7"); 
  
Motor *lm = robot->getMotor("leftMotor");
Motor *rm = robot->getMotor("rightMotor");
  
void setup(){
  lm->setPosition(INFINITY);
  rm->setPosition(INFINITY);
  lm->setVelocity(0);
  rm->setVelocity(0);
}

void squre_check(){
  int count = 0;
  for (int i=0;i<=s;i++){
    if (i){
      count +=1;
    }
    else{
      break;
    }
  }
  if (((count)<3) && (act<3) && (count==act)){
    insquare=false;
  }
}


void sensor_check(){
  act=0;
  for (int i=0;i<=s;i++){
    values[i]=0;
  }
  v1 = 1-int((s1->getValue())/t);
  values[0]=v1;
  v2 = 1-int((s2->getValue())/t);
  values[1]=v2;
  v3 = 1-int((s3->getValue())/t);
  values[2]=v3;
  v4 = 1-int((s4->getValue())/t);
  values[3]=v4;
  v5 = 1-int((s5->getValue())/t);
  values[4]=v5;
  v6 = 1-int((s6->getValue())/t);
  values[5]=v6;
  v7 = 1-int((s7->getValue())/t);
  values[6]=v7;
  v8 = 1-int((s8->getValue())/t);
  values[7]=v8;
  for (int i=0;i<=s;i++){
    act+=values[i];
  }
  
}

void speedset(){
  lm->setVelocity(ls);
  rm->setVelocity(rs);
}


void endcheck(){
 
  for (int i=0;i<40;i++){
    sensor_check();
    if (act==0){
      robot->step(time);
      }else{
      break;
      }
    if (i>38){
      end=1;
      ls=0;
      rs=0;
      speedset();
    }
    else{
    end=0;
    }
   }
}

void err(){

  error=0;
  pos=0;  
  if (act!=0){
    for (int i=0;i<=s;i++){
      pos+=values[i]*(i+1);
    }
    
   error = (4.5-((float)pos/act)); 
  }
  else{
      error=0;
  }
}


void pid(){
  err();
  p = error;
  i=i+error;
  d=error-lasterror;
  lasterror = error;
  speed=kp*p+ki*i+kd*d;

  ls =base;
  rs =base;   
  ls=base+speed;
  rs=base-speed;
    
  if (ls>max){
    ls=max;
  }
  if (rs>max){
    rs=max;
  }
  if (ls<0){
    ls=0;
  }
  if (rs<0){
    rs=0;
  }
  speedset();
      
}


int main(int argc, char **argv) {

  setup();
  
  s1->enable(time1);
  s2->enable(time1);
  s3->enable(time1);
  s4->enable(time1);
  s5->enable(time1);
  s6->enable(time1);
  s7->enable(time1);
  s8->enable(time1);
  
  while (robot->step(time) != -1) {
    sensor_check();
    if (insquare){
      squre_check();
      ls =base;
      rs =base;
    }
    else{
      if (end){
        break;
      }
      pid();
      endcheck();
      }
  }
      
  delete robot;
  return 0;
}