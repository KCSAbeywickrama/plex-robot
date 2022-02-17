#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define time 16
#define time1 16
#define max 6.28
#define base 3


bool whiteline= false;
double kp=4;
double ki=0.001;
double kd=0.2;
double p=0;
double i=0;
double d=0;
double error;
double speed;
double rs;
double ls;
double lasterror = 0;

double etime=0;
double eetime=0;
double ttime =0;
int et=0;
int tc =0;
int pos;
int t=800;
int s = 8;
int act=0;
int values [8];
int value[8];
int red=0;
bool end=0;

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

void sensor_check(){
  act=0;
  
  values[0] = 1-int((s1->getValue())/t);
  values[0] = 1-int((s1->getValue())/t);
  values[1] = 1-int((s2->getValue())/t);
  values[1] = 1-int((s2->getValue())/t);
  values[2] = 1-int((s3->getValue())/t);
  values[2] = 1-int((s3->getValue())/t);
  values[3] = 1-int((s4->getValue())/t);
  values[4] = 1-int((s5->getValue())/t);
  values[5] = 1-int((s6->getValue())/t);
  values[6] = 1-int((s7->getValue())/t);
  values[7] = 1-int((s8->getValue())/t);


  for (int i=0;i<=s;i++){
    act+=values[i];
  }
  
    for (int i=0;i<=s;i++){
    std::cout <<values[i]<< std::endl; 
  }
  std::cout <<"**************"<< std::endl; 
}

void speedset(){
  lm->setVelocity(ls);
  rm->setVelocity(rs);
}


void endcheck(){
     
  if(act>5){
 
  if((robot->getTime()-etime)<0.1){
  et+=1;
     std::cout <<et<< std::endl; 
  if (et>4){
  eetime=robot->getTime();
  while((robot->getTime()-eetime)<1){
  if (act==0){
  end=1;
  }
  }}}

  else{
  et=0;
  }
    etime=robot->getTime();
  }}


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
  if (act>3){
  error=0;
  }
}

void tcheck(){
if(values[7]==1 && values[0]==0){
if((robot->getTime()-ttime)<0.1){
        tc+=1;
        if(tc>2){
          tc=0;
          if (red){
   
        while(values[7]==1){
       sensor_check();
      robot->step(time);
      ls=0;
        rs=base;
        speedset();
        }
  
}
}
}
else{
tc=0;
}
ttime=robot->getTime();
}
}

void pid(){
  err();
  p = error;
  i=i+error;
  d=error-lasterror;
  lasterror = error;
  speed=kp*p+ki*i+kd*d;
  speedset();
 
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
    if (0){
      ls=0;
      rs=0;
      speedset();
      break;
     }else{
        tcheck();
        pid();
        //endcheck();
        
      }
      
  }
  delete robot;
  return 0;
}