/**********************************************************************************************
 * Arduino PID Library - Version 1.0.2
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * Modified by Jason Melvin to include feedforward and adjustable windup
 * 
 * This Code is licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <PID_v1.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, double Kf, int ControllerDirection)
{
	PID::SetOutputLimits(0, 255);	//default output limit corresponds to 
						//the arduino pwm limits
						
	windupI = outMax; 		// default windup limit is outMax
	sampleTime = 100;		// default Controller Sample Time is 0.1 seconds
	ff_zero = 0; 			// default feedforward zero point is 0.0 (for temp control, set to ambient)
	
	PID::SetControllerDirection(ControllerDirection);
	PID::SetTunings(Kp, Ki, Kd, Kf);

	lastTime = millis()-sampleTime;				
	inAuto = false;
	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed
 **********************************************************************************/ 
void PID::Compute()
{
   if(!inAuto) return;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=sampleTime)
   {
      /*Compute all the working error variables*/
	double input = *myInput; 				// read current input condition
	double error = *mySetpoint - input; 		// error is difference between setpoint and input 
	iTerm+= (ki * error); 					// add additional error to the integral term
	iTerm = constrain( iTerm , -windupI , windupI );	// limit the integral term to +/- windup parameter
	double dInput = (input - lastInput); 		// derivative is based on change in the input
 
      /*Compute PID Output*/
	double output = kp * error + iTerm - kd * dInput + kf * (*mySetpoint - ff_zero);
      
	output = constrain( output , outMin , outMax );	// limit output to within min/max
	*myOutput = output;				// write the current output
 
      /*Remember some variables for next time*/
	lastInput = input;					// last input is for computing the derivative term
	lastTime = now;					// last time is for determining when to recompute
   }
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd, double Kf)
{
   if (Kp<0 || Ki<0 || Kd<0 || Kf<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd; dispKf = Kf;
   
   double sampleTimeInSec = ((double)sampleTime)/1000;  
   kp = Kp;
   ki = Ki * sampleTimeInSec;
   kd = Kd / sampleTimeInSec;
   kf = Kf;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
      kf = (0 - kf);
   }
}
  
/* SetsampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetSampleTime(int NewsampleTime)
{
   if (NewsampleTime > 0)
   {
      double ratio  = (double)NewsampleTime
                      / (double)sampleTime;
      ki *= ratio;
      kd /= ratio;
      sampleTime = (unsigned long)NewsampleTime;
   }
}

/* SetWindupI(...)********************
*  Sets the windup limit for the integral term,
*  which is otherwise limited to outMax
***************************************/
void PID::SetWindupI(double limit)
{
	if (limit > 0) windupI = limit;
}

/* SetFF_zero(...)********************
*  Sets the zero point for the feedforward term,
*  which is otherwise 0.0
***************************************/
void PID::SetFFzero(double zeropoint)
{
	ff_zero = zeropoint;
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   if (outMax) windupI *= Max / outMax;
   outMin = Min;
   outMax = Max;
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(iTerm > outMax) iTerm= outMax;
	   else if(iTerm < outMin) iTerm= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::Initialize()
{
   iTerm = *myOutput;
   lastInput = *myInput;
   if(iTerm > windupI) iTerm = windupI;
   else if(iTerm < outMin) iTerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
	kp = (0 - kp);
	ki = (0 - ki);
	kd = (0 - kd);
	kf = (0 - kf);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
double PID::GetKf(){ return dispKf;}
double PID::GetWi(){ return windupI;}
double PID::GetFFzero(){ return ff_zero;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}
