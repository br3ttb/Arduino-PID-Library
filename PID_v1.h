#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.1.1

  //Constants used in some of the functions below
  const char AUTOMATIC    {1}; // gives better compiler-errors than #defines
  const char MANUAL	    {0};
  const char DIRECT       {0};
  const char REVERSE      {1};

class PID
{

  public:

  //commonly used functions **************************************************************************
    PID( double* const, double* const, double* const,        // * constructor.  links the PID to the Input, Output, and
        const double, const double, const double, const char);     //   Setpoint.  Initial tuning parameters are also set here

    void SetMode(const char Mode);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(const double, const double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



  //available but not commonly used functions ********************************************************
    void SetTunings(const double, const double,       // * While most users will set the tunings once in the
                    const double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void SetControllerDirection(const char);	  // * Sets the Direction, or "Action" of the controller. DIRECT
										  //   means the output will increase when error is positive. REVERSE
										  //   means the opposite.  it's very unlikely that this will be needed
										  //   once it is set in the constructor.
    void SetSampleTime(const int);              // * sets the frequency, in Milliseconds, with which
                                          //   the PID calculation is performed.  default is 100



  //Display functions ****************************************************************
/* Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
    inline double GetKp()       { return  dispKp; };
    inline double GetKi()       { return  dispKi;};
    inline double GetKd()       { return  dispKd;};
    inline char   GetMode()     { return  inAuto ? AUTOMATIC : MANUAL;};
    inline char   GetDirection(){ return controllerDirection;};


  private:
	void Initialize();

	PID(const PID&);                    // declaration only for copy constructor
	PID& operator=(const PID&);         // declaration only for copy assignment --> make it uncopyable

	double dispKp;				// * we'll hold on to the tuning parameters in user-entered
	double dispKi;				//   format for display purposes
	double dispKd;				//

	double kp;                  // * (P)roportional Tuning Parameter
    double ki;                  // * (I)ntegral Tuning Parameter
    double kd;                  // * (D)erivative Tuning Parameter

	char  controllerDirection;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

	unsigned long lastTime;
	double ITerm, lastInput;

	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto;
};
#endif

