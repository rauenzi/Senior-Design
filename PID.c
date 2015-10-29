/**********************************************************************************************
 * Arduino PID Library - Version 1.0.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/


#include <PID_v1.h>

/*Setup (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID_Setup(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
{
	
    PID_myOutput = Output;
    PID_myInput = Input;
    PID_mySetpoint = Setpoint;
	PID_inAuto = false;
	
	PID_SetOutputLimits(0, 100);				//default output limit corresponds to 
												//the arduino pwm limits
    PID_SetControllerDirection(ControllerDirection);
    PID_SetTunings(Kp, Ki, Kd);		
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PID_Compute()
{
   if(!PID_inAuto) return false;
  /*Compute all the working error variables*/
  double input = *PID_myInput;
  double error = *PID_mySetpoint - input;
  PID_ITerm+= (PID_ki * error);
  if(PID_ITerm > PID_outMax) PID_ITerm= PID_outMax;
  else if(PID_ITerm < PID_outMin) PID_ITerm= PID_outMin;
  double dInput = (input - PID_lastInput);

  /*Compute PID Output*/
  double output = PID_kp * error + PID_ITerm- PID_kd * dInput;
  
  if(output > PID_outMax) output = PID_outMax;
  else if(output < PID_outMin) output = PID_outMin;
  *PID_myOutput = output;
  
  /*Remember some variables for next time*/
  PID_lastInput = input;
  return true;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID_SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   PID_dispKp = Kp; PID_dispKi = Ki; PID_dispKd = Kd;
   
   double SampleTimeInSec = ((double)PID_SampleTime)/1000;  
   PID_kp = Kp;
   PID_ki = Ki * SampleTimeInSec;
   PID_kd = Kd / SampleTimeInSec;
 
  if(PID_controllerDirection ==REVERSE)
   {
      PID_kp = (0 - PID_kp);
      PID_ki = (0 - PID_ki);
      PID_kd = (0 - PID_kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID_SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)PID_SampleTime;
      PID_ki *= ratio;
      PID_kd /= ratio;
      PID_SampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   PID_outMin = Min;
   PID_outMax = Max;
 
   if(PID_inAuto)
   {
	   if(*PID_myOutput > PID_outMax) *PID_myOutput = PID_outMax;
	   else if(*PID_myOutput < PID_outMin) *PID_myOutput = PID_outMin;
	 
	   if(PID_ITerm > PID_outMax) PID_ITerm= PID_outMax;
	   else if(PID_ITerm < PID_outMin) PID_ITerm= PID_outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID_SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !PID_inAuto)
    {  /*we just went from manual to auto*/
        PID_Initialize();
    }
    PID_inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID_Initialize()
{
   PID_ITerm = *PID_myOutput;
   PID_lastInput = *PID_myInput;
   if(PID_ITerm > PID_outMax) PID_ITerm = PID_outMax;
   else if(PID_ITerm < PID_outMin) PID_ITerm = PID_outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(int Direction)
{
   if(PID_inAuto && Direction !=PID_controllerDirection)
   {
	  PID_kp = (0 - PID_kp);
      PID_ki = (0 - PID_ki);
      PID_kd = (0 - PID_kd);
   }   
   PID_controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID_GetKp(){ return  PID_dispKp; }
double PID_GetKi(){ return  PID_dispKi;}
double PID_GetKd(){ return  PID_dispKd;}
int PID_GetMode(){ return  PID_inAuto ? AUTOMATIC : MANUAL;}
int PID_GetDirection(){ return PID_controllerDirection;}

