#ifndef PID_h
#define PID_h

class PID {

public: 
  PID (double*, double*, double*, double, double, double,int );

  void updatePID(int);

  void setOutLim(double, double);

  void setParam(double, double, double);

  void compute();

  double getKp();
  double getKi();
  double getKd();
  
private:

  void Initialize();

  double kp;                 
  double ki;                  
  double kd;                  
    
  int sampleTime;

  double *myInput;              
  double *myOutput;            
  double *mySetpoint;          
        
  unsigned long currentTime, lastTime;
  double ITerm, lastInput;

  double outMin, outMax;
  
  
};

#endif
