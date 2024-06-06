#include "Arduino.h"
#include "ArduinoUnitTests.h"
#include <PID_v1.h>

unittest(direct) {

  // Define Variables we'll be connecting to
  double Setpoint, Input, Output;

  // Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);

  // initialize the variables we're linked to
  Input = 50;
  Setpoint = 100;

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  for (int i = 0; i < 1000; i++) {
    delay(200);
    if (myPID.Compute()) {
      Input = Input + (Output - Input) / 25.6;
    }
  }
  // std::cout << "Input = " << Input << "; Output = " << Output << std::endl;
  assertEqual(Setpoint, round(Input));
}

unittest(reverse) {

  // Define Variables we'll be connecting to
  double Setpoint, Input, Output;

  // Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE);

  // initialize the variables we're linked to
  Input = 50;
  Setpoint = 100;

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  for (int i = 0; i < 1000; i++) {
    delay(200);
    if (myPID.Compute()) {
      Input = Input + (Input - Output) / 25.6;
    }
  }
  // std::cout << "Input = " << Input << "; Output = " << Output << std::endl;
  assertEqual(Setpoint, round(Input));
}

unittest(mode) {

  // Define Variables we'll be connecting to
  double Setpoint, Input, Output;

  // Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE);

  // initialize the variables we're linked to
  Input = 50;
  Setpoint = 100;

  // turn the PID on
  myPID.SetMode(MANUAL);
  for (int i = 0; i < 1000; i++) {
    delay(200);
    if (myPID.Compute()) {
      Input = Input + (Input - Output) / 25.6;
    }
  }
  // std::cout << "Input = " << Input << "; Output = " << Output << std::endl;
  assertEqual(50, round(Input));
}

unittest(getFunctions) {

  // Define Variables we'll be connecting to
  double Setpoint, Input, Output;

  // Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE);

  // initialize the variables we're linked to
  Input = 50;
  Setpoint = 100;

  // turn the PID on
  myPID.SetMode(AUTOMATIC);

  // std::cout << "Input = " << Input << "; Output = " << Output << std::endl;
  assertEqual(2, myPID.GetKp());
  assertEqual(5, myPID.GetKi());
  assertEqual(1, myPID.GetKd());
  assertEqual(REVERSE, myPID.GetDirection());
  assertEqual(AUTOMATIC, myPID.GetMode());
}

unittest(sampleTimeWorks) {

  // Define Variables we'll be connecting to
  double Setpoint, Input, Output;

  // Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE);

  // initialize the variables we're linked to
  Input = 50;
  Setpoint = 100;

  bool flag = false;
  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  for (int i = 0; i < 1000; i++) {
    delay(200);
    if (myPID.Compute()) {
      flag = true;
      Input = Input + (Input - Output) / 25.6;
    }
  }
  // std::cout << "Input = " << Input << "; Output = " << Output << std::endl;
  assertTrue(flag);
}

unittest(sampleTimeNotWorks) {

  // Define Variables we'll be connecting to
  double Setpoint, Input, Output;

  // Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, REVERSE);

  // initialize the variables we're linked to
  Input = 50;
  Setpoint = 100;

  bool flag = false;
  // turn the PID on
  myPID.SetMode(MANUAL);
  for (int i = 0; i < 1000; i++) {
    delay(199);
    if (myPID.Compute()) {
      flag = true;
      Input = Input + (Input - Output) / 25.6;
    }
  }
  // std::cout << "Input = " << Input << "; Output = " << Output << std::endl;
  assertFalse(flag);
}

unittest_main()
