#ifndef PID_H
#define PID_H

typedef struct
{
  double dState;       // Last position input
  double iState;       // Integrator state
  double iMax, iMin;  
  // Maximum and minimum allowable integrator state
  double iGain,     // integral gain
  pGain,     // proportional gain
  dGain;     // derivative gain
  //
  double output;
  double setpoint;
} 
SPid;

double UpdatePID(SPid * pid, double position) {
  double pTerm, dTerm, iTerm;
  double error = position - pid->setpoint;
  //
  pTerm = pid->pGain * error;   
  // calculate the proportional term
  // calculate the integral state with appropriate limiting
  pid->iState += error;
  if (pid->iState > pid->iMax) pid->iState = pid->iMax;
  else if (pid->iState < pid->iMin) pid->iState = pid->iMin;
  iTerm = pid->iGain * pid->iState;  // calculate the integral term
  dTerm = pid->dGain * (position - pid->dState);
  pid->dState = position;
  pid->output = pTerm + iTerm - dTerm;
  return pid->output;
}


#endif

