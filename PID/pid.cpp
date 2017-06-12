
#if ARDUINO >= 100
    #include "Arduino.h"
#else 
    #include "WProgram.h"
#endif

#include "pid.h"

/*Constructor (...)*********************************************************
 *    Setting up the parameter defined by the users
 *   
 ***************************************************************************/
 
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int sampleTime)
{
  
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;

    PID::setOutLim(0, 255);       
    PID::setParam(Kp, Ki, Kd);

    currentTime = millis();
           
}

/*updatePID (...)*********************************************************
 *    Update the PID based on the sample rate defined by the user.
 *   
 ***************************************************************************/
void PID::updatePID(int sampleTime) {

  if(sampleTime <= 0 ){
    sampleTime = 100;                //The default sample rate 0.1 sec 
  }

  if((currentTime-lastTime) > sampleTime)
    {
     
     //include PID update here 
     lastTime = currentTime;
    }  
}
/*updatePID (...)*********************************************************
 *  User set the output limit of the PID
 *   
 ***************************************************************************/
void PID::setOutLim(double Min, double Max) {
  outMax = Max;
  outMin = Min; 
}
/*setParam (...)*********************************************************
 *  User set the parameter of the PID controller. 
 *  The Kp Ki and Kd
 *   
 ***************************************************************************/
void PID::setParam(double Kp, double Ki, double Kd) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
}

void PID::compute() {
  
      double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 

      double output = kp * error + ITerm- kd * dInput;
     
      if(output > outMax){ 
         output = outMax;
      }else if(output < outMin) {
        output = outMin;
         *myOutput = output;
   
      lastInput = input;
    
}
  
}

double PID::getKp(){ return  kp; }
double PID::getKi(){ return  ki;}
double PID::getKd(){ return  kd;}


