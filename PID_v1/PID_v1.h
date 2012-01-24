#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.2

class PID
{

  public:

  //Constants used in some of the functions below
  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1

  //commonly used functions **************************************************************************
	PID(double*, double*, double*,			// * constructor.  links the PID to the Input, Output, and 
		double, double, double, double, int);	//   Setpoint.  Initial tuning parameters are also set here
								//   kp, ki, kd, kf, direction
	
	void SetMode(int Mode);				// * sets PID to either Manual (0) or Auto (non-0)

	void Compute();	// * performs the PID calculation.  it should be
				//   called every time loop() cycles. ON/OFF and
				//   calculation frequency can be set using SetMode
				//   SetSampleTime respectively

	void SetOutputLimits(double, double);	//clamps the output to a specific range. 0-255 by default, but
							//it's likely the user will want to change this depending on
							//the application

  //available but not commonly used functions ********************************************************
	void SetTunings(double, double,	// * While most users will set the tunings once in the 
		double, double);		//   constructor, this function gives the user the option
						//   of changing tunings during runtime for Adaptive control
	
	void SetControllerDirection(int);	// * Sets the Direction, or "Action" of the controller. DIRECT
						//   means the output will increase when error is positive. REVERSE
						//   means the opposite.  it's very unlikely that this will be needed
						//   once it is set in the constructor.
	
	void SetSampleTime(int);		// * sets the frequency, in Milliseconds, with which 
						//   the PID calculation is performed.  default is 100
										  
	void SetWindupI(double); // set the integral windup limit (default is outMax)
	
	void SetFFzero(double); // set the zero point for the feedforward term (default is 0; for temp control, set to ambient)
										  
  //Display functions ****************************************************************
	double GetKp();	// These functions query the pid for interal values.
	double GetKi();	//  they were created mainly for the pid front-end,
	double GetKd();	// where it's important to know what is actually 
	double GetKf();	// inside the PID.
	double GetWi();
	double GetFFzero();
	int GetMode();
	int GetDirection();

  private:
	void Initialize();
	
	double dispKp;	// * we'll hold on to the tuning parameters in user-entered 
	double dispKi;	//   format for display purposes
	double dispKd;
	double dispKf;
    
	double kp;		// * (P)roportional Tuning Parameter
	double ki;		// * (I)ntegral Tuning Parameter
	double kd;		// * (D)erivative Tuning Parameter
	double kf;		// * (F)eedforward turning parameter
	double ff_zero; 	//  * feedforward zero point

	int controllerDirection;

	double *myInput;		// * Pointers to the Input, Output, and Setpoint variables
	double *myOutput;		//   This creates a hard link between the variables and the 
	double *mySetpoint;		//   PID, freeing the user from having to constantly tell us
					//   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	double iTerm, lastInput;
	double windupI;		// windup limit for integral term
	int sampleTime;
	double outMin, outMax;
	bool inAuto;
};
#endif